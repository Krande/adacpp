#include "gtest/gtest.h"
#include "../../src/fem/simple_mesh.h"
#include <fstream>

TEST(GmshTest, CreatesFile) {
    std::string filename = "test_mesh.msh";

    // Call the function to create the mesh file
    simple_gmesh(filename);

    // Check if the file was created
    std::ifstream file(filename);
    ASSERT_TRUE(file.good()) << "File " << filename << " was not created";

    // Check if the file is not empty
    file.seekg(0, std::ios::end);
    ASSERT_GT(file.tellg(), 0) << "File " << filename << " is empty";

    // Cleanup the file
    file.close();
    std::remove(filename.c_str());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
