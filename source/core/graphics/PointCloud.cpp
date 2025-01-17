#include "PointCloud.h"

#include "core/graphics/PointCloud.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
#include <thread>

namespace n2m::graphics {
void PointCloud::upload (const std::vector<GLfloat>& vertexData,
    int componentsPerVertex,
    const std::vector<unsigned int>& indices) {
    // init vao vbo ebo

    glGenVertexArrays (1, &VAO);
    glGenBuffers (1, &VBO);
    glGenBuffers (1, &EBO);

    // Ensure componentsPerVertex is 3 for glm::vec3 storage
    if (componentsPerVertex != 3) {
        throw std::invalid_argument (
            "upload() only supports 3 components per vertex for now.");
    }

    // Convert glm::vec3 to PCL PointCloud
    m_pcl_cloud_points->points.reserve (
        vertexData.size () / componentsPerVertex);


    // Convert vertexData to cachedVertices (std::vector<glm::vec3>)
    vertices.clear ();
    for (size_t i = 0; i < vertexData.size (); i += componentsPerVertex) {
        vertices.emplace_back (
            vertexData[i],
            vertexData[i + 1],
            vertexData[i + 2]
            );

        // Populate pcl data structure wit hthe points
        m_pcl_cloud_points->points.emplace_back (vertexData[i],
            vertexData[i + 1],
            vertexData[i + 2]);
    }

    m_pcl_cloud_points->width = static_cast<uint32_t> (m_pcl_cloud_points->
        points.
        size ());
    m_pcl_cloud_points->height   = 1; // Unorganized
    m_pcl_cloud_points->is_dense = true;

    this->indices.clear ();
    this->indices = indices;

    vertexCount = static_cast<int> (vertexData.size () / componentsPerVertex);

    glBindVertexArray (VAO);
    // upload vertex data
    glBindBuffer (GL_ARRAY_BUFFER, VBO);
    glBufferData (GL_ARRAY_BUFFER,
        vertexCount * componentsPerVertex * sizeof (GLfloat),
        vertexData.data (),
        GL_STATIC_DRAW);

    // If we have indices, use them
    if (!indices.empty ()) {
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData (GL_ELEMENT_ARRAY_BUFFER,
            indices.size () * sizeof (unsigned int),
            indices.data (),
            GL_STATIC_DRAW);

        indicesCount = static_cast<GLsizei> (indices.size ());
    }

    // Setup vertex attribute (location = 0 for positions)
    glVertexAttribPointer (0,
        componentsPerVertex,
        GL_FLOAT,
        GL_FALSE,
        componentsPerVertex * sizeof (GLfloat),
        (void*)0
        );
    glEnableVertexAttribArray (0);

    // Unbind
    glBindVertexArray (0);
    glBindBuffer (GL_ARRAY_BUFFER, 0);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
}


void PointCloud::draw () const {
    Geometry::draw ();
}


void PointCloud::removeOutliers (
    int meanK,
    double stddevMul) {
    if (!m_pcl_cloud_points || m_pcl_cloud_points->empty ()) {
        throw std::runtime_error (
            "removeOutliers: input cloud is null or empty.");
    }

    // Create a StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (m_pcl_cloud_points);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stddevMul);

    sor.filter (*this->m_pcl_cloud_points);
}


void PointCloud::estimateNormals (
    int kSearchNormals) {
    if (!m_pcl_cloud_points || m_pcl_cloud_points->empty ()) {
        throw std::runtime_error (
            "estimateNormals: input cloud is null or empty.");
    }

    // Create a KdTree for neighbor searches
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (
        new pcl::search::KdTree<pcl::PointXYZ> ());
    kdtree->setInputCloud (m_pcl_cloud_points);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setNumberOfThreads (std::thread::hardware_concurrency ());
    pcl::PointCloud<pcl::Normal>::Ptr
        normals (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setInputCloud (m_pcl_cloud_points);
    normalEstimation.setSearchMethod (kdtree);
    normalEstimation.setKSearch (kSearchNormals);

    // Compute normals
    normalEstimation.compute (*this->m_pcl_cloud_normals);

    // Concatenate XYZ and normals
    pcl::concatenateFields (*this->m_pcl_cloud_points,
        *this->m_pcl_cloud_normals,
        *this->m_pcl_cloud_points_normals);
}
} // namespace n2m::graphics
