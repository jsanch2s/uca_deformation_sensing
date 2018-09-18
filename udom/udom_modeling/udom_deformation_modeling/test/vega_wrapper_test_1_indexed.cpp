/*
 * Copyright 2016 Université Clermont Auvergne (UCA)
 *
 * Author: Jose Sanchez
 *
 */

#include <string>
#include <vector>
#include <udom_deformation_modeling/vega_wrapper.h>
#include <gtest/gtest.h>

namespace
{
class VegaWrapperTest1Indexed : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        my_interface.reset(new VegaWrapper(
            mesh_filename, constrained_nodes, timestep, damping_mass, damping_stiffness));
  }

    double accuracy = 0.0001;
    const std::string mesh_filename = "test_mesh_1_indexed.veg";
    std::vector<int> constrained_nodes;
    double timestep = 0.02;
    double damping_mass = 0.0;
    double damping_stiffness = 0.01;
    udom_modeling_msgs::Mesh ros_mesh;

    std::shared_ptr<VegaWrapper> my_interface;
};

TEST_F(VegaWrapperTest1Indexed, test_init_vertices)
{
    my_interface->init_mesh(ros_mesh);

    EXPECT_NEAR(0.0, ros_mesh.vertices[0].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[0].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[0].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[1].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[1].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[1].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[2].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[2].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[2].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[3].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[3].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[3].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[4].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[4].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[4].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[5].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[5].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[5].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[6].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[6].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[6].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[7].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[7].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[7].z, accuracy);
}

TEST_F(VegaWrapperTest1Indexed, test_init_elements)
{
    my_interface->init_mesh(ros_mesh);

    std::vector<int> element_1{0, 2, 1, 4};
    std::vector<int> element_2{6, 5, 4, 2};
    std::vector<int> element_3{6, 7, 5, 3};
    std::vector<int> element_4{3, 5, 1, 2};

    for (int ii = 0; ii < element_1.size(); ++ii) {
      EXPECT_EQ(element_1[ii], ros_mesh.tetrahedra[0].vertex_indices[ii]);
    }
    for (int ii = 0; ii < element_2.size(); ++ii) {
      EXPECT_EQ(element_2[ii], ros_mesh.tetrahedra[1].vertex_indices[ii]);
    }
    for (int ii = 0; ii < element_3.size(); ++ii) {
      EXPECT_EQ(element_3[ii], ros_mesh.tetrahedra[2].vertex_indices[ii]);
    }
    for (int ii = 0; ii < element_4.size(); ++ii) {
      EXPECT_EQ(element_4[ii], ros_mesh.tetrahedra[3].vertex_indices[ii]);
    }
}

TEST_F(VegaWrapperTest1Indexed, test_mesh_dofs)
{
    int mesh_dofs = my_interface->mesh_dofs();
    EXPECT_EQ(24, mesh_dofs);
}

TEST_F(VegaWrapperTest1Indexed, test_extract_points)
{
    std::vector<int> indices = {1, 5, 8};
    visualization_msgs::Marker points;
    my_interface->extract_points(indices, points, 1);
    EXPECT_EQ(3, points.points.size());
    EXPECT_NEAR(0.0, points.points[0].x, accuracy);
    EXPECT_NEAR(0.0, points.points[0].y, accuracy);
    EXPECT_NEAR(0.0, points.points[0].z, accuracy);
    EXPECT_NEAR(0.0, points.points[1].x, accuracy);
    EXPECT_NEAR(0.0, points.points[1].y, accuracy);
    EXPECT_NEAR(1.0, points.points[1].z, accuracy);
    EXPECT_NEAR(0.0, points.points[2].x, accuracy);
    EXPECT_NEAR(1.0, points.points[2].y, accuracy);
    EXPECT_NEAR(0.0, points.points[2].z, accuracy);
}

TEST_F(VegaWrapperTest1Indexed, test_extract_points_various_dofs)
{
    std::vector<int> indices = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    visualization_msgs::Marker points;
    my_interface->extract_points(indices, points, 1);
    EXPECT_EQ(3, points.points.size());
    EXPECT_NEAR(0.0, points.points[0].x, accuracy);
    EXPECT_NEAR(0.0, points.points[0].y, accuracy);
    EXPECT_NEAR(0.0, points.points[0].z, accuracy);
    EXPECT_NEAR(0.0, points.points[1].x, accuracy);
    EXPECT_NEAR(0.0, points.points[1].y, accuracy);
    EXPECT_NEAR(1.0, points.points[1].z, accuracy);
    EXPECT_NEAR(0.0, points.points[2].x, accuracy);
    EXPECT_NEAR(1.0, points.points[2].y, accuracy);
    EXPECT_NEAR(0.0, points.points[2].z, accuracy);
}

TEST_F(VegaWrapperTest1Indexed, test_get_displacements)
{
    int mesh_dofs = my_interface->mesh_dofs();
    std::vector<double> displacements(mesh_dofs, 0.0);
    my_interface->init_mesh(ros_mesh);
    my_interface->get_displacements(displacements);

    EXPECT_EQ(24, displacements.size());
}

TEST_F(VegaWrapperTest1Indexed, test_displacements_update)
{
    std::vector<double> displacements = {
        0.1, 0.2, 0.3, 0.4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3
    };

    my_interface->init_mesh(ros_mesh);
    my_interface->update_mesh(ros_mesh, displacements);

    EXPECT_NEAR(0.1, ros_mesh.vertices[0].x, accuracy);
    EXPECT_NEAR(0.2, ros_mesh.vertices[0].y, accuracy);
    EXPECT_NEAR(0.3, ros_mesh.vertices[0].z, accuracy);

    EXPECT_NEAR(0.4, ros_mesh.vertices[1].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[1].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[1].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[2].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[2].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[2].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[3].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[3].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[3].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[4].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[4].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[4].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[5].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[5].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[5].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[6].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[6].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[6].z, accuracy);

    EXPECT_NEAR(2.0, ros_mesh.vertices[7].x, accuracy);
    EXPECT_NEAR(3.0, ros_mesh.vertices[7].y, accuracy);
    EXPECT_NEAR(4.0, ros_mesh.vertices[7].z, accuracy);
}

TEST_F(VegaWrapperTest1Indexed, test_reset_mesh)
{
    std::vector<double> displacements = {
        0.1, 0.2, 0.3, 0.4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3
    };

    my_interface->init_mesh(ros_mesh);
    my_interface->update_mesh(ros_mesh, displacements);
    my_interface->reset_mesh(ros_mesh);

    EXPECT_NEAR(0.0, ros_mesh.vertices[0].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[0].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[0].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[1].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[1].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[1].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[2].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[2].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[2].z, accuracy);

    EXPECT_NEAR(0.0, ros_mesh.vertices[3].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[3].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[3].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[4].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[4].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[4].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[5].x, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[5].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[5].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[6].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[6].y, accuracy);
    EXPECT_NEAR(0.0, ros_mesh.vertices[6].z, accuracy);

    EXPECT_NEAR(1.0, ros_mesh.vertices[7].x, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[7].y, accuracy);
    EXPECT_NEAR(1.0, ros_mesh.vertices[7].z, accuracy);
}
}   // namespace


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
