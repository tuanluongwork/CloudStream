#pragma once

#include "core/PointCloud.h"
#include "utils/ThreadPool.h"
#include <memory>

namespace CloudStream {

class PointCloudProcessor {
public:
    PointCloudProcessor(ThreadPool* thread_pool = nullptr);
    ~PointCloudProcessor() = default;
    
    // Filtering operations
    PointCloud::Ptr removeStatisticalOutliers(PointCloud::Ptr input,
                                             int k_neighbors = 50,
                                             float std_dev_threshold = 1.0f);
    
    PointCloud::Ptr voxelGridFilter(PointCloud::Ptr input,
                                   float voxel_size);
    
    PointCloud::Ptr radiusOutlierRemoval(PointCloud::Ptr input,
                                        float radius,
                                        int min_neighbors);
    
    // Normal estimation
    void computeNormals(PointCloud::Ptr cloud,
                       float search_radius = 0.1f);
    
    // Transformations
    PointCloud::Ptr transform(PointCloud::Ptr input,
                             const glm::mat4& transformation);
    
    // Segmentation
    std::vector<PointCloud::Ptr> euclideanClustering(PointCloud::Ptr input,
                                                     float cluster_tolerance,
                                                     size_t min_cluster_size,
                                                     size_t max_cluster_size);
    
    // Feature extraction
    struct Features {
        glm::vec3 centroid;
        glm::vec3 eigenvalues;
        glm::mat3 eigenvectors;
        float planarity;
        float sphericity;
        float linearity;
    };
    
    Features computeFeatures(PointCloud::Ptr cloud);
    
private:
    ThreadPool* thread_pool_;
    bool owns_thread_pool_;
};

} // namespace CloudStream 