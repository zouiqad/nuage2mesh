#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "Geometry.h"
#include <memory.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>


namespace n2m::graphics {
class PointCloud : public Geometry {
public:
    PointCloud() {
        m_pcl_cloud_points = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        m_pcl_cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal> >();
        m_pcl_cloud_points_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();
    }

    virtual ~PointCloud() = default;

    void upload(const std::vector<GLfloat> &vertexData,
                int componentsPerVertex,
                const std::vector<unsigned int> &indices = {}) override;

    void draw() const override;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > getPCLCloudPoints() const {
        return m_pcl_cloud_points;
    }

    std::shared_ptr<pcl::PointCloud<pcl::Normal> > getPCLCloudNormals() const {
        return m_pcl_cloud_normals;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointNormal> > getPCLCloudPointsNormals() const {
        return m_pcl_cloud_points_normals;
    }

    void removeOutliers(
        int meanK = 50,
        double stddevMul = 1.0);

    void estimateNormals(int kSearchNormals = 20);

private:
    std::shared_ptr<pcl::PointCloud<pcl::PointNormal> > m_pcl_cloud_points_normals;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > m_pcl_cloud_points;
    std::shared_ptr<pcl::PointCloud<pcl::Normal> > m_pcl_cloud_normals;
};
} // namespace n2m::graphics

#endif
