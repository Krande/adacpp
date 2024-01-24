//
// Just a quick experiment. Untested and probably not working
//

#include "FEMSolver2d.h"
#include <vector>
#include <Eigen/Dense>

// 2D node coordinates
typedef Eigen::Vector2d Node;

// Triangle element defined by three node indices
typedef Eigen::Vector3i Element;

// 2D FEM solver
class FEMSolver2D {
public:
    Eigen::VectorXd solve(const std::vector<Node>& nodes,
                          const std::vector<Element>& elements,
                          const Eigen::VectorXd& forces,
                          const std::vector<int>& fixedDofs,
                          const Eigen::VectorXd& fixedValues) {
        // Create the global stiffness matrix and force vector
        int nDofs = nodes.size() * 2;  // 2 degrees of freedom (x and y) for each node
        Eigen::MatrixXd K = Eigen::MatrixXd::Zero(nDofs, nDofs);
        Eigen::VectorXd F = forces;

        // Loop over all elements
        for (const auto& element : elements) {
            // Compute the local stiffness matrix
            Eigen::MatrixXd K_local = compute_local_stiffness(element, nodes);

            // Assemble the local stiffness matrix into the global stiffness matrix
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    K(element[i], element[j]) += K_local(i, j);
                }
            }
        }

        // Apply the displacement boundary conditions
        for (size_t i = 0; i < fixedDofs.size(); ++i) {
            int dof = fixedDofs[i];
            double value = fixedValues[i];

            // Modify the row corresponding to the fixed degree of freedom
            K.row(dof).setZero();
            K(dof, dof) = 1.0;
            F(dof) = value;
        }

        // Solve the system K * u = F
        Eigen::VectorXd u = K.fullPivLu().solve(F);

        return u;
    }

private:
    Eigen::MatrixXd compute_local_stiffness(const Element& element, const std::vector<Node>& nodes) {
        // Initialize the stiffness matrix for the current element
        Eigen::MatrixXd K_local = Eigen::MatrixXd::Zero(6, 6);

        // Compute the area of the triangle
        double area = 0.5 * std::abs(nodes[element(0)].x() * (nodes[element(1)].y() - nodes[element(2)].y()) +
                                     nodes[element(1)].x() * (nodes[element(2)].y() - nodes[element(0)].y()) +
                                     nodes[element(2)].x() * (nodes[element(0)].y() - nodes[element(1)].y()));

        // Compute the B matrix (strain-displacement matrix)
        Eigen::Matrix<double, 3, 6> B;
        for (int i = 0; i < 3; ++i) {
            B(0, 2 * i) = (nodes[element((i + 1) % 3)].y() - nodes[element((i + 2) % 3)].y()) / (2 * area);
            B(1, 2 * i + 1) = (nodes[element((i + 2) % 3)].x() - nodes[element((i + 1) % 3)].x()) / (2 * area);
            B(2, 2 * i) = B(1, 2 * i + 1);
            B(2, 2 * i + 1) = B(0, 2 * i);
        }

        // Define the D matrix (material matrix), assuming isotropic linear elastic material and plane stress
        double E = 210000.0;  // Young's modulus
        double nu = 0.3;  // Poisson's ratio
        Eigen::Matrix3d D;
        D << 1, nu, 0,
                nu, 1, 0,
                0, 0, 0.5 * (1 - nu);
        D *= E / (1 - nu * nu);

        // Compute the local stiffness matrix
        K_local = B.transpose() * D * B * area;

        return K_local;
    }

};



