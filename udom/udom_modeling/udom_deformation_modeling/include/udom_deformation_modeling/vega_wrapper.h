/*
 * Copyright 2016 Université Clermont Auvergne (UCA)
 *
 * Author: Jose Sanchez
 *
 */

#ifndef UDOM_DEFORMATION_MODELING_VEGA_WRAPPER_H
#define UDOM_DEFORMATION_MODELING_VEGA_WRAPPER_H

#include <string>
#include <vector>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <udom_modeling_msgs/Mesh.h>
#include <udom_modeling_msgs/MeshTetrahedron.h>
#include <vega_fem/volumetricMeshLoader.h>
#include <vega_fem/corotationalLinearFEM.h>
#include <vega_fem/corotationalLinearFEMForceModel.h>
#include <vega_fem/generateMassMatrix.h>
#include <vega_fem/implicitBackwardEulerSparse.h>
#include <vega_fem/vec3d.h>

/**
 * Defines functions to use the Vega FEM library such as loading a mesh,
 * creating a deformable model, applying forces to it and obtaining the new
 * positions of the mesh.
 */
class VegaWrapper final
{
    public:
        /**
         * Constructor.
         *
         * @param input_filename Filename of the volumetric mesh.
         *
         * @param constrained_nodes List of the mesh's nodes that are constrained for movement.
         *
         * @param timestep Duration of the timestep (in seconds).
         *
         * @param damping_mass Mass damping coefficient.
         *
         * @param damping_stiffness Stiffness damping coefficient.
         */
        explicit VegaWrapper(
            const std::string &input_filename, const std::vector<int> &constrained_nodes,
            double timestep, double damping_mass, double damping_stiffness);

        /// Destructor.
        ~VegaWrapper();

    private:
        /// Copy constructor.
        VegaWrapper(const VegaWrapper &other);
        /// Assignment operator
        VegaWrapper &operator=(const VegaWrapper &other);

    public:
        /**
         * Sets the external forces on the mesh as specified by the 'force' vector and
         * computes the deformation of the mesh.
         *
         * @param forces Vector of forces applied to each node on the mesh. The vector
         * specifies the force on the X, Y and Z direction for each node in order.
         */
        void apply_forces(std::vector<double> &forces);

        /**
         * Fills the 'displacements' vector with the new positions of the mesh's nodes
         * caused by the applied external forces.
         *
         * @param displacements Vector to be filled with the displacements of the mesh's nodes.
         */
        void get_displacements(std::vector<double> &displacements);

        /**
         * Returns a set points, for the specified indices, as a ROS marker message.
         *
         * @param indices Indices of the specified point to extract from the mesh.
         *
         * @param points The actual points (x, y, z) as a ROS marker message.
         *
         * @param index_start Where the indexes start (e.g. 0 for zero-based numbering).
         */
        void extract_points(
            std::vector<int> &indices, visualization_msgs::Marker &points, int index_start);

        /**
         * Returns the initial mesh as a ROS message.
         *
         * @param mesh ROS mesh to be filled with the initial mesh model.
         */
        void init_mesh(udom_modeling_msgs::Mesh &mesh);

        /**
         * Resets the mesh to its original undeformed shape.
         *
         * @param mesh ROS mesh to be reset to its initial mesh model.
         */
        void reset_mesh(udom_modeling_msgs::Mesh &mesh);

        /**
         * Returns the updated mesh as a ROS message.
         *
         * @param mesh ROS mesh to be filled with the updated mesh model.
         *
         * @param displacements A vector with the displacements of the vertices.
         */
        void update_mesh(udom_modeling_msgs::Mesh &mesh, std::vector<double> &displacements);

        /// Returns the degrees of freedom of the mesh, i.e. the number of nodes times three.
        int mesh_dofs() { return degrees_of_freedom_; }

    private:
        /// Number of elements the mesh has.
        int num_elements;
        /// Number of elements per vertex.
        int elements_per_vertex;
        /// The indices representing the mesh's elements.
        int *elements;
        /// Number of vertices the mesh has.
        int num_vertices;
        /// The initial vertices positions with an X, Y, Z ordering.
        double *vertices;

        /// Time step duration (in seconds).
        double timestep_;

        /// Number of degrees of freedom of the mesh (i.e. 3 * number of nodes).
        int degrees_of_freedom_;

        /**
         * Constrained vertices of the mesh. Each constrained node must specify
         * its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14:
         * constrained_nodes_ = {12, 13, 14, 30, 31, 32, 42, 43, 44};
         */
        std::vector<int> constrained_nodes_;

        /** @name Damping parameters.
         *  (tangential) Rayleigh damping.
         */
        ///@{
        /// Mass damping coefficient, "underwater"-like damping.
        double damping_mass_;
        /// Stiffness damping coefficient, (primarily) high-frequency damping.
        double damping_stiffness_;

        /**
        * Integrator to timestep the deformable model in time. It connects a force model
        * to the deformable model in order to apply forces to the model and compute the
        * deformation.
        */
        std::unique_ptr<ImplicitBackwardEulerSparse> implicit_integrator_;

        /// Volumetric mesh of the object.
        std::unique_ptr<VolumetricMesh> volumetric_mesh_;
};

#endif  // UDOM_DEFORMATION_MODELING_VEGA_WRAPPER_H
