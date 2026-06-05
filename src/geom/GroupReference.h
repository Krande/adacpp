#ifndef NANO_OCCT_GROUPREFERENCE_H
#define NANO_OCCT_GROUPREFERENCE_H

class GroupReference {
public:
    int node_id;
    int start;    // first index into the combined index buffer
    int length;   // number of indices for this group (3 * triangles)
    int vstart;   // first vertex into the combined position buffer
    int vlength;  // number of vertices for this group

    GroupReference(int node_id, int start, int length, int vstart, int vlength);
};

#endif //NANO_OCCT_GROUPREFERENCE_H
