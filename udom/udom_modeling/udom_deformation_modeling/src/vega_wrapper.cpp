/*
 * Copyright 2016 Université Clermont Auvergne (UCA)
 *
 * Author: Jose Sanchez
 *
 */

#include <string>
#include <vector>
#include <set>
#include <udom_deformation_modeling/vega_wrapper.h>


VegaWrapper::VegaWrapper(
    const std::string &input_filename, const std::vector<int> &constrained_nodes,
    double timestep, double damping_mass, double damping_stiffness) :
    timestep_(timestep), damping_mass_(damping_mass), damping_stiffness_(damping_stiffness)
{
    volumetric_mesh_ = std::unique_ptr<VolumetricMesh>(VolumetricMeshLoader::load(
        input_filename.c_str()));

    if (volumetric_mesh_ == NULL)
        throw std::runtime_error("Failed to load mesh.");

    TetMesh *tet_mesh_;
    if (volumetric_mesh_->getElementType() == VolumetricMesh::TET) {
        tet_mesh_ = static_cast<TetMesh*>(volumetric_mesh_.get());
    }
    else
    {
        throw std::runtime_error("Not a tet mesh.");
    }

    degrees_of_freedom_ =  3 * tet_mesh_->getNumVertices();
    constrained_nodes_ = constrained_nodes;

    // Get mesh information.
    volumetric_mesh_->exportMeshGeometry(
        &num_vertices, &vertices, &num_elements, &elements_per_vertex, &elements);

    // Deformable model and force model.
    CorotationalLinearFEM *deformable_model_ = new CorotationalLinearFEM(tet_mesh_);
    ForceModel *force_model_ = new CorotationalLinearFEMForceModel(deformable_model_);

    // Create consistent (non-lumped) mass matrix.
    SparseMatrix *mass_matrix_;
    GenerateMassMatrix::computeMassMatrix(tet_mesh_, &mass_matrix_, true);

    // Initialize the integrator.
    implicit_integrator_.reset(new ImplicitBackwardEulerSparse
    (
            degrees_of_freedom_, timestep_, mass_matrix_, force_model_,
            constrained_nodes_.size(), constrained_nodes_.data(), damping_mass_,
            damping_stiffness_));
}

VegaWrapper::~VegaWrapper()
{
}

void VegaWrapper::apply_forces(std::vector<double> &forces)
{
    implicit_integrator_->SetExternalForcesToZero();
    implicit_integrator_->SetExternalForces(forces.data());
    implicit_integrator_->DoTimestep();
}

void VegaWrapper::get_displacements(std::vector<double> &displacements)
{
    implicit_integrator_->GetqState(displacements.data());
}

void VegaWrapper::extract_points(
    std::vector<int> &indices, visualization_msgs::Marker &points, int index_start)
{
    std::vector<double> vertices_vec(vertices, vertices + (num_vertices * 3));
    std::set<int> done_vertices;

    for (auto ii : indices)
    {
        int vertex = (ii - index_start) / 3;
        // Test if the index is unique (since each index has three degrees of freedom).
        if (done_vertices.find(vertex) == done_vertices.end()) {
            done_vertices.insert(vertex);
            geometry_msgs::Point point;

            int index = vertex * 3;
            point.x = vertices_vec[index];
            point.y = vertices_vec[index + 1];
            point.z = vertices_vec[index + 2];

            points.points.push_back(point);
        }
    }
}

void VegaWrapper::init_mesh(udom_modeling_msgs::Mesh &mesh)
{
    std::vector<double> vertices_vec(vertices, vertices + (num_vertices * 3));
    for (std::size_t ii = 0; ii != num_vertices; ++ii) {
        int index = ii * 3;
        geometry_msgs::Point point;

        point.x = vertices_vec[index];
        point.y = vertices_vec[index + 1];
        point.z = vertices_vec[index + 2];

        mesh.vertices.push_back(point);
    }

    std::vector<int> elements_vec(elements, elements + num_elements * elements_per_vertex);
    for (std::size_t ii = 0; ii != num_elements; ++ii) {
        int index = ii * elements_per_vertex;
        udom_modeling_msgs::MeshTetrahedron tetrahedra;

        tetrahedra.vertex_indices[0] = elements_vec[index];
        tetrahedra.vertex_indices[1] = elements_vec[index + 1];
        tetrahedra.vertex_indices[2] = elements_vec[index + 2];
        tetrahedra.vertex_indices[3] = elements_vec[index + 3];

        mesh.tetrahedra.push_back(tetrahedra);
    }
}

void VegaWrapper::reset_mesh(udom_modeling_msgs::Mesh &mesh)
{
    implicit_integrator_->ResetToRest();
    implicit_integrator_->SetState(new double[num_vertices * 3]());

    std::vector<double> vertices_vec(vertices, vertices + (num_vertices * 3));
    for (std::size_t ii = 0; ii != num_vertices; ++ii) {
        int index = ii * 3;

        mesh.vertices.at(ii).x = vertices_vec.at(index);
        mesh.vertices.at(ii).y = vertices_vec.at(index + 1);
        mesh.vertices.at(ii).z = vertices_vec.at(index + 2);
    }
}

void VegaWrapper::update_mesh(udom_modeling_msgs::Mesh &mesh, std::vector<double> &displacements)
{
    std::vector<double> vertices_vec(vertices, vertices + (num_vertices * 3));
    for (std::size_t ii = 0; ii != displacements.size() / 3; ++ii) {
        int index = ii * 3;

        mesh.vertices.at(ii).x = vertices_vec.at(index) + displacements.at(index);
        mesh.vertices.at(ii).y = vertices_vec.at(index + 1) + displacements.at(index + 1);
        mesh.vertices.at(ii).z = vertices_vec.at(index + 2) + displacements.at(index + 2);
    }
}
