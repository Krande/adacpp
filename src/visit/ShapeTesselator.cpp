// Copyright 2011 Fotios Sioutis (sfotis@gmail.com)
//
//This file is part of pythonOCC.
//
//pythonOCC is free software: you can redistribute it and/or modify
//it under the terms of the GNU Lesser General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//pythonOCC is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public License
//along with pythonOCC.  If not, see <http://www.gnu.org/licenses/>.

//---------------------------------------------------------------------------
#include "ShapeTesselator.h"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iomanip>
//---------------------------------------------------------------------------
#include <TopExp_Explorer.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopoDS.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopExp.hxx>
#include <BRepTools.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Face.hxx>
#include <Precision.hxx>

//---------------------------------------------------------------------------
ShapeTesselator::ShapeTesselator(TopoDS_Shape aShape):
  myShape(aShape),
  locVertexcoord(NULL),
  locNormalcoord(NULL),
  locTriIndices(NULL),
  computed(false)
{
    ComputeDefaultDeviation();
}

void ShapeTesselator::Compute(bool compute_edges, float mesh_quality, bool parallel)
{
    if (!computed) {
      Tesselate(compute_edges, mesh_quality, parallel);
    }

    computed=true;
}

ShapeTesselator::~ShapeTesselator()
{
    if (locVertexcoord)
      delete [] locVertexcoord;

    if (locNormalcoord)
      delete [] locNormalcoord;

    if (locTriIndices)
      delete [] locTriIndices;

    for (std::vector<aedge*>::iterator edgeit = edgelist.begin(); edgeit != edgelist.end(); ++edgeit) {
      aedge* edge = *edgeit;
      if (edge) {
        if (edge->vertex_coord)
          delete[] edge->vertex_coord;

        delete edge;
        *edgeit = NULL;
      }
    }
    edgelist.clear();
}

//---------------------------------------------------------------------------
void ShapeTesselator::SetDeviation(Standard_Real aDeviation)
{
    myDeviation = aDeviation;
}


//---------------------------------------------------------------------------
void ShapeTesselator::Tesselate(bool compute_edges, float mesh_quality, bool parallel)
{
    TopExp_Explorer ExpFace;
    // clean shape to remove any previous tringulation
    BRepTools::Clean(myShape);
    //Triangulate
    BRepMesh_IncrementalMesh(myShape, myDeviation*mesh_quality, false, 0.5*mesh_quality, parallel);


    for (ExpFace.Init(myShape, TopAbs_FACE); ExpFace.More(); ExpFace.Next()) {
        Standard_Integer validFaceTriCount = 0;
        Standard_Integer invalidFaceTriCount = 0;
        Standard_Integer invalidNormalCount = 0;
        TopLoc_Location aLocation;

        const TopoDS_Face& myFace = TopoDS::Face(ExpFace.Current());
        Handle(Poly_Triangulation) myT = BRep_Tool::Triangulation(myFace, aLocation);

        if (myT.IsNull()) {
            invalidFaceTriCount++;
            continue;
        }

        aface *this_face = new aface;

        //write vertex buffer
        this_face->vertex_coord = new Standard_Real[myT->NbNodes() * 3];
        this_face->number_of_coords = myT->NbNodes();
        for (int i = 1; i <= myT->NbNodes(); i++) {
          gp_Pnt p = myT->Node(i).Transformed(aLocation).XYZ();

          int idx = (i - 1) * 3;
          this_face->vertex_coord[idx] = p.X();
          this_face->vertex_coord[idx + 1] = p.Y();
          this_face->vertex_coord[idx + 2] = p.Z();
        }
        // compute normals and write normal buffer, using the uv nodes
        if (myT->HasUVNodes()) {
            BRepGProp_Face prop(myFace);
            this_face->normal_coord = new Standard_Real[myT->NbNodes() * 3];
            this_face->number_of_normals = myT->NbNodes();

            for (int i = 1; i <= myT->NbNodes(); ++i) {
                const gp_Pnt2d& uv_pnt = myT->UVNode(i);
                gp_Pnt p; gp_Vec n;
                prop.Normal(uv_pnt.X(),uv_pnt.Y(),p,n);
                if (n.SquareMagnitude() > Precision::SquareConfusion()) {
                    n.Normalize();
                }
                else {
                    n.SetCoord(0., 0., 0.);
                }
                if (myFace.Orientation() == TopAbs_INTERNAL) {
                    n.Reverse();
                }
                int idx = (i - 1) * 3;
                this_face->normal_coord[idx] = n.X();
                this_face->normal_coord[idx + 1] = n.Y();
                this_face->normal_coord[idx + 2] = n.Z();
            }
        }
        else {
            invalidNormalCount++;
        }

        //write triangle buffer
        TopAbs_Orientation orient = myFace.Orientation();
        const Standard_Integer trianglesNb = myT->NbTriangles();
        this_face->tri_indexes  = new int[trianglesNb * 3];
        for (Standard_Integer nt = 1; nt <= trianglesNb; nt++) {
            Standard_Integer n0 , n1 , n2;
            myT->Triangle(nt).Get(n0, n1, n2);
            if (orient == TopAbs_REVERSED) {
                Standard_Integer tmp=n1;
                n1 = n2;
                n2 = tmp;
            }
            this_face->tri_indexes[validFaceTriCount * 3 + 0] = n0;
            this_face->tri_indexes[validFaceTriCount * 3 + 1] = n1;
            this_face->tri_indexes[validFaceTriCount * 3 + 2] = n2;
            validFaceTriCount++;
        }

        this_face->number_of_triangles = validFaceTriCount;
        this_face->number_of_invalid_triangles = invalidFaceTriCount;
        this_face->number_of_invalid_normals = invalidNormalCount;
        facelist.push_back(this_face);
    }
    JoinPrimitives();

    if (compute_edges) ComputeEdges();
}


//---------------------------INTERFACE---------------------------------------
void ShapeTesselator::ComputeDefaultDeviation()
{
    // This method automatically computes precision from the bounding box of the shape
    Bnd_Box aBox;
    Standard_Real aXmin,aYmin ,aZmin ,aXmax ,aYmax ,aZmax;

    //calculate the bounding box
    BRepBndLib::Add(myShape, aBox);
    aBox.Get(aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);

    Standard_Real adeviation = std::max(aXmax-aXmin, std::max(aYmax-aYmin, aZmax-aZmin)) * 2e-2 ;
    myDeviation = adeviation;
}

void ShapeTesselator::ComputeEdges()
{
  TopLoc_Location aTrsf;

  // clear current data
  std::vector<aedge*>::iterator it;
  for (it = edgelist.begin(); it != edgelist.end(); ++it) {
    if (*it) {
      if ((*it)->vertex_coord)
        delete[] (*it)->vertex_coord;

      delete *it;
      *it = NULL;
    }
  }
  edgelist.clear();
  // get indexed map of edges
  TopTools_IndexedMapOfShape M;
  TopExp::MapShapes(myShape, TopAbs_EDGE, M);

  // explore all boundary edges
  TopTools_IndexedDataMapOfShapeListOfShape edgeMap;
  TopExp::MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, edgeMap);

  for (int iEdge = 1 ; iEdge <= edgeMap.Extent (); iEdge++) {

    // skip free edges, might be the case if the shape passed to
    // the tesselator is a Compound
    const TopTools_ListOfShape& faceList = edgeMap.FindFromIndex(iEdge);

    if (faceList.Extent() == 0) {
      printf("Skipped free edge during shape tesselation/edges computation.\n");
      continue;
    }

    // take one of the shared edges and get edge triangulation
    const TopoDS_Edge& anEdge = TopoDS::Edge(M(iEdge));
    gp_Trsf myTransf;
    TopLoc_Location aLoc;

    // triangulate the edge
    Handle(Poly_Polygon3D) aPoly = BRep_Tool::Polygon3D(anEdge, aLoc);


    aedge* theEdge = new aedge;
    Standard_Integer nbNodesInFace;

    // edge triangulation successfull
    if (!aPoly.IsNull ()) {
        if (!aLoc.IsIdentity()) myTransf = aLoc.Transformation();
        nbNodesInFace = aPoly->NbNodes();
        theEdge->number_of_coords = nbNodesInFace;
        theEdge->vertex_coord = new Standard_Real[nbNodesInFace * 3 * sizeof(Standard_Real)];

        const TColgp_Array1OfPnt& Nodes = aPoly->Nodes();
        for (Standard_Integer i=0;i < nbNodesInFace;i++) {
            gp_Pnt V = Nodes(i+1);
            V.Transform(myTransf);
            theEdge->vertex_coord[i*3 + 0] = V.X();
            theEdge->vertex_coord[i*3 + 1] = V.Y();
            theEdge->vertex_coord[i*3 + 2] = V.Z();
        }
    }

    // edge triangulation failed
    else {
        const TopoDS_Face& aFace = TopoDS::Face(edgeMap.FindFromIndex(iEdge).First());
        // take the face's triangulation instead
        Handle(Poly_Triangulation) aPolyTria = BRep_Tool::Triangulation(aFace, aLoc);
        if (!aLoc.IsIdentity()) myTransf = aLoc.Transformation();
        // this holds the indices of the edge's triangulation to the actual points
        Handle(Poly_PolygonOnTriangulation) aPoly = BRep_Tool::PolygonOnTriangulation(anEdge, aPolyTria, aLoc);
        if (aPoly.IsNull()) continue; // polygon does not exist

        // getting size and create the array
        nbNodesInFace = aPoly->NbNodes();
        theEdge->number_of_coords = nbNodesInFace;
        theEdge->vertex_coord = new Standard_Real[nbNodesInFace * 3 * sizeof(Standard_Real)];

        const TColStd_Array1OfInteger& indices = aPoly->Nodes();

        // go through the index array
        for (Standard_Integer i=1;i <= aPoly->NbNodes();i++) {
            gp_Pnt V = aPolyTria->Node(indices(i)).Transformed(myTransf).XYZ();
            int idx = (i - 1) * 3;
            theEdge->vertex_coord[idx] = V.X();
            theEdge->vertex_coord[idx + 1] = V.Y();
            theEdge->vertex_coord[idx + 2] = V.Z();
          }
      }
   edgelist.push_back(theEdge);
   }
}

void ShapeTesselator::EnsureMeshIsComputed()
{
  // this method ensures that the mesh is computed before returning any
  // related data
  if (!computed) {
    printf("The mesh is not computed. Currently computing with default parameters ...");
    Compute(true, 1.0, false);
    printf("done\n");
    printf("Call explicitely the Compute method to set the parameters value.\n");
  }
}

std::string formatFloatNumber(float f)
{
  // returns string representation of the float number f.
  // set epsilon to 1e-3
  float epsilon = 1e-3;
  std::stringstream formatted_float;
  if (std::abs(f) < epsilon) {
    f = 0.;
  }
  formatted_float << f;
  return formatted_float.str();
}

std::vector<float> ShapeTesselator::GetVerticesPositionAsTuple()
{
  EnsureMeshIsComputed();
  // create the vector and allocate memory
  std::vector<float> vertices_position;
  vertices_position.reserve(tot_triangle_count);
  // loop over tertices
  for (int i=0;i<tot_triangle_count;i++) {
      int pID = locTriIndices[(i * 3) + 0] * 3;
      vertices_position.push_back(locVertexcoord[pID]);
      vertices_position.push_back(locVertexcoord[pID+1]);
      vertices_position.push_back(locVertexcoord[pID+2]);
      // Second vertex
      int qID = locTriIndices[(i * 3) + 1] * 3;
      vertices_position.push_back(locVertexcoord[qID]);
      vertices_position.push_back(locVertexcoord[qID+1]);
      vertices_position.push_back(locVertexcoord[qID+2]);
      // Third vertex
      int rID = locTriIndices[(i * 3) + 2] * 3;
      vertices_position.push_back(locVertexcoord[rID]);
      vertices_position.push_back(locVertexcoord[rID+1]);
      vertices_position.push_back(locVertexcoord[rID+2]);
    }
  return vertices_position;
}

std::vector<float> ShapeTesselator::GetNormalsAsTuple()
{
  EnsureMeshIsComputed();
  // create the vector and allocate memory
  std::vector<float> normals;
  normals.reserve(tot_triangle_count);
  // loop over normals
  for (int i=0;i<tot_triangle_count;i++) {
      int pID = locTriIndices[(i * 3) + 0] * 3;
      normals.push_back(locNormalcoord[pID]);
      normals.push_back(locNormalcoord[pID+1]);
      normals.push_back(locNormalcoord[pID+2]);
      // Second normal
      int qID = locTriIndices[(i * 3) + 1] * 3;
      normals.push_back(locNormalcoord[qID]);
      normals.push_back(locNormalcoord[qID+1]);
      normals.push_back(locNormalcoord[qID+2]);
      // Third normal
      int rID = locTriIndices[(i * 3) + 2] * 3;
      normals.push_back(locNormalcoord[rID]);
      normals.push_back(locNormalcoord[rID+1]);
      normals.push_back(locNormalcoord[rID+2]);
    }
  return normals;
}

//---------------------------------------------------------------------------
Standard_Real* ShapeTesselator::VerticesList()
{
  EnsureMeshIsComputed();
  return locVertexcoord;
}
//---------------------------------------------------------------------------
Standard_Real* ShapeTesselator::NormalsList()
{
  EnsureMeshIsComputed();
  return locNormalcoord;
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjGetInvalidTriangleCount()
{
  EnsureMeshIsComputed();
  return tot_invalid_triangle_count;
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjGetTriangleCount()
{
  EnsureMeshIsComputed();
  return tot_triangle_count;
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjGetVertexCount()
{
  EnsureMeshIsComputed();
  return tot_vertex_count;
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjGetNormalCount()
{
  EnsureMeshIsComputed();
  return tot_normal_count;
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjGetInvalidNormalCount()
{
  EnsureMeshIsComputed();
  return tot_invalid_normal_count;
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjGetEdgeCount()
{
  EnsureMeshIsComputed();
  return edgelist.size();
}
//---------------------------------------------------------------------------
Standard_Integer ShapeTesselator::ObjEdgeGetVertexCount(int iEdge)
{
  EnsureMeshIsComputed();
  aedge* edge = edgelist.at(iEdge);
  if (!edge) {
    return 0;
  }
  return edge->number_of_coords;
}
//---------------------------------------------------------------------------
void ShapeTesselator::GetVertex(int ivert, float& x, float& y, float& z)
{
  EnsureMeshIsComputed();
  x = locVertexcoord[ivert*3 + 0];
  y = locVertexcoord[ivert*3 + 1];
  z = locVertexcoord[ivert*3 + 2];
}
//---------------------------------------------------------------------------
void ShapeTesselator::GetNormal(int ivert, float& x, float& y, float& z)
{
  EnsureMeshIsComputed();
  x = locNormalcoord[ivert*3 + 0];
  y = locNormalcoord[ivert*3 + 1];
  z = locNormalcoord[ivert*3 + 2];
}
//---------------------------------------------------------------------------
void ShapeTesselator::GetTriangleIndex(int triangleIdx, int &v1, int &v2, int &v3)
{
  EnsureMeshIsComputed();
  v1 = locTriIndices[3*triangleIdx + 0];
  v2 = locTriIndices[3*triangleIdx + 1];
  v3 = locTriIndices[3*triangleIdx + 2];
}
//---------------------------------------------------------------------------
void ShapeTesselator::GetEdgeVertex(int iEdge, int ivert, float &x, float &y, float &z)
{
  EnsureMeshIsComputed();
  aedge* e = edgelist.at(iEdge);
  if (!e) {
    return;
  }

  x = e->vertex_coord[3*ivert + 0];
  y = e->vertex_coord[3*ivert + 1];
  z = e->vertex_coord[3*ivert + 2];
}
//---------------------------------------------------------------------------
void ShapeTesselator::ObjGetTriangle(int trianglenum, int *vertices, int *normals)
{
  EnsureMeshIsComputed();
  int pID = locTriIndices[(trianglenum * 3) + 0] * 3;
  int qID = locTriIndices[(trianglenum * 3) + 1] * 3;
  int rID = locTriIndices[(trianglenum * 3) + 2] * 3;

  vertices[0] = pID;
  vertices[1] = qID;
  vertices[2] = rID;

  normals[0] = pID;
  normals[1] = qID;
  normals[2] = rID;
}
//---------------------------------------------------------------------------
//---------------------------------HELPERS-----------------------------------
//---------------------------------------------------------------------------
void ShapeTesselator::JoinPrimitives()
{
  int obP = 0;
  int obN = 0;
  int obTR = 0;

  int advance = 0;

  tot_triangle_count = 0;
  tot_invalid_triangle_count = 0;
  tot_vertex_count = 0;
  tot_normal_count = 0;
  tot_invalid_normal_count = 0;

  std::vector<aface*>::iterator anIterator = facelist.begin();

  while (anIterator != facelist.end()) {

    aface* myface = *anIterator;

    tot_triangle_count =  tot_triangle_count + myface->number_of_triangles;
    tot_invalid_triangle_count =  tot_invalid_triangle_count + myface->number_of_invalid_triangles;
    tot_vertex_count = tot_vertex_count + myface->number_of_coords;
    tot_normal_count = tot_normal_count + myface->number_of_normals;
    tot_invalid_normal_count = tot_invalid_normal_count + myface->number_of_invalid_normals;

    ++anIterator;
  }

  locTriIndices= new Standard_Integer[tot_triangle_count * 3 ];
  locVertexcoord = new Standard_Real[tot_vertex_count * 3 ];
  locNormalcoord = new Standard_Real[tot_normal_count * 3 ];

  anIterator = facelist.begin();
  while (anIterator != facelist.end()) {
    aface* myface = *anIterator;
    for (int x = 0; x < myface->number_of_coords; x++) {
      locVertexcoord[(obP * 3) + 0] = myface->vertex_coord[(x * 3) + 0];
      locVertexcoord[(obP * 3) + 1] = myface->vertex_coord[(x * 3) + 1];
      locVertexcoord[(obP * 3) + 2] = myface->vertex_coord[(x * 3) + 2];
      obP++;
    }

    for (int x = 0; x < myface->number_of_normals; x++) {
      locNormalcoord[(obN * 3) + 0] = myface->normal_coord[(x * 3) + 0];
      locNormalcoord[(obN * 3) + 1] = myface->normal_coord[(x * 3) + 1];
      locNormalcoord[(obN * 3) + 2] = myface->normal_coord[(x * 3) + 2];
      obN++;
    }

    for (int x = 0; x < myface->number_of_triangles; x++) {
      locTriIndices[(obTR * 3) + 0] = myface->tri_indexes[(x * 3) + 0] + advance - 1;
      locTriIndices[(obTR * 3) + 1] = myface->tri_indexes[(x * 3) + 1] + advance - 1;
      locTriIndices[(obTR * 3) + 2] = myface->tri_indexes[(x * 3) + 2] + advance - 1;
      obTR++;
    }

    advance = obP;

    delete [] myface->vertex_coord;
    myface->vertex_coord = NULL;

    delete [] myface->normal_coord;
    myface->normal_coord = NULL;

    delete [] myface->tri_indexes;
    myface->tri_indexes = NULL;

    delete myface;
    myface = NULL;

    ++anIterator;
  }
}