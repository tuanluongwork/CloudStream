#include "core/PointCloudProcessor.h"
#include "utils/Timer.h"
#include "utils/Logger.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace CloudStream {

PointCloudProcessor::PointCloudProcessor(ThreadPool* thread_pool)
    : thread_pool_(thread_pool)
    , owns_thread_pool_(thread_pool == nullptr) {
    if (owns_thread_pool_) {
        thread_pool_ = new ThreadPool(std::thread::hardware_concurrency());
    }
}

PointCloudProcessor::~PointCloudProcessor() {
    if (owns_thread_pool_ && thread_pool_) {
        delete thread_pool_;
    }
}

PointCloud::Ptr PointCloudProcessor::removeStatisticalOutliers(PointCloud::Ptr input,
                                                              int k_neighbors,
                                                              float std_dev_threshold) {
    if (!input || input->empty()) {
        return std::make_shared<PointCloud>();
    }
    
    ScopedTimer timer("Statistical Outlier Removal");
    Logger::instance().info("Removing statistical outliers from {} points", input->size());
    
    auto output = std::make_shared<PointCloud>();
    output->reserve(input->size());
    
    // Build KD-tree for efficient neighbor search (simplified version)
    std::vector<std::vector<float>> distances(input->size());
    
    // Parallel computation of neighbor distances
    if (thread_pool_) {
        std::vector<std::future<void>> futures;
        size_t chunk_size = input->size() / thread_pool_->getNumThreads();
        
        for (size_t i = 0; i < input->size(); i += chunk_size) {
            size_t end = std::min(i + chunk_size, input->size());
            
            futures.push_back(thread_pool_->enqueue([&, i, end]() {
                for (size_t idx = i; idx < end; ++idx) {
                    const auto& point = (*input)[idx];
                    std::vector<float> point_distances;
                    
                    // Find k nearest neighbors (brute force for simplicity)
                    for (size_t j = 0; j < input->size(); ++j) {
                        if (j != idx) {
                            float dist = glm::distance(point.position, (*input)[j].position);
                            point_distances.push_back(dist);
                        }
                    }
                    
                    // Sort and keep k nearest
                    std::partial_sort(point_distances.begin(), 
                                    point_distances.begin() + std::min(k_neighbors, (int)point_distances.size()),
                                    point_distances.end());
                    
                    point_distances.resize(std::min(k_neighbors, (int)point_distances.size()));
                    distances[idx] = std::move(point_distances);
                }
            }));
        }
        
        for (auto& future : futures) {
            future.wait();
        }
    }
    
    // Compute mean and standard deviation of distances
    std::vector<float> mean_distances;
    mean_distances.reserve(input->size());
    
    for (const auto& dist_vec : distances) {
        if (!dist_vec.empty()) {
            float mean = std::accumulate(dist_vec.begin(), dist_vec.end(), 0.0f) / dist_vec.size();
            mean_distances.push_back(mean);
        }
    }
    
    float global_mean = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0f) / mean_distances.size();
    float variance = 0.0f;
    
    for (float dist : mean_distances) {
        variance += (dist - global_mean) * (dist - global_mean);
    }
    variance /= mean_distances.size();
    float std_dev = std::sqrt(variance);
    
    // Filter points
    float threshold = global_mean + std_dev_threshold * std_dev;
    
    for (size_t i = 0; i < input->size(); ++i) {
        if (i < mean_distances.size() && mean_distances[i] < threshold) {
            output->addPoint((*input)[i]);
        }
    }
    
    Logger::instance().info("Removed {} outliers", input->size() - output->size());
    return output;
}

PointCloud::Ptr PointCloudProcessor::voxelGridFilter(PointCloud::Ptr input, float voxel_size) {
    if (!input || input->empty() || voxel_size <= 0.0f) {
        return input;
    }
    
    ScopedTimer timer("Voxel Grid Filter");
    Logger::instance().info("Downsampling {} points with voxel size {}", input->size(), voxel_size);
    
    // Use the built-in downsample method
    auto output = std::make_shared<PointCloud>(*input);
    output->downsample(voxel_size);
    
    Logger::instance().info("Downsampled to {} points", output->size());
    return output;
}

PointCloud::Ptr PointCloudProcessor::radiusOutlierRemoval(PointCloud::Ptr input,
                                                         float radius,
                                                         int min_neighbors) {
    if (!input || input->empty()) {
        return std::make_shared<PointCloud>();
    }
    
    ScopedTimer timer("Radius Outlier Removal");
    auto output = std::make_shared<PointCloud>();
    output->reserve(input->size());
    
    // For each point, count neighbors within radius
    for (size_t i = 0; i < input->size(); ++i) {
        const auto& point = (*input)[i];
        int neighbor_count = 0;
        
        for (size_t j = 0; j < input->size(); ++j) {
            if (i != j) {
                float dist = glm::distance(point.position, (*input)[j].position);
                if (dist <= radius) {
                    neighbor_count++;
                    if (neighbor_count >= min_neighbors) {
                        break;
                    }
                }
            }
        }
        
        if (neighbor_count >= min_neighbors) {
            output->addPoint(point);
        }
    }
    
    Logger::instance().info("Radius outlier removal: {} -> {} points", input->size(), output->size());
    return output;
}

void PointCloudProcessor::computeNormals(PointCloud::Ptr cloud, float search_radius) {
    if (!cloud || cloud->empty()) {
        return;
    }
    
    ScopedTimer timer("Normal Estimation");
    Logger::instance().info("Computing normals for {} points", cloud->size());
    
    // Parallel normal computation
    if (thread_pool_) {
        thread_pool_->parallel_for(cloud->begin(), cloud->end(),
            [&](Point& point) {
                // Find neighbors within search radius
                std::vector<glm::vec3> neighbors;
                
                for (const auto& other : *cloud) {
                    float dist = glm::distance(point.position, other.position);
                    if (dist > 0.0f && dist <= search_radius) {
                        neighbors.push_back(other.position);
                    }
                }
                
                if (neighbors.size() >= 3) {
                    // Compute covariance matrix using Eigen
                    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
                    glm::vec3 centroid(0.0f);
                    
                    // Include the point itself
                    neighbors.push_back(point.position);
                    
                    // Compute centroid
                    for (const auto& pos : neighbors) {
                        centroid += pos;
                    }
                    centroid /= static_cast<float>(neighbors.size());
                    
                    // Build covariance matrix
                    for (const auto& pos : neighbors) {
                        glm::vec3 diff = pos - centroid;
                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 3; ++j) {
                                covariance(i, j) += diff[i] * diff[j];
                            }
                        }
                    }
                    covariance /= static_cast<float>(neighbors.size());
                    
                    // Compute eigenvectors
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
                    Eigen::Vector3f eigenvalues = solver.eigenvalues();
                    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
                    
                    // The normal is the eigenvector with smallest eigenvalue
                    int min_idx = 0;
                    if (eigenvalues[1] < eigenvalues[min_idx]) min_idx = 1;
                    if (eigenvalues[2] < eigenvalues[min_idx]) min_idx = 2;
                    
                    point.normal = glm::vec3(eigenvectors(0, min_idx),
                                           eigenvectors(1, min_idx),
                                           eigenvectors(2, min_idx));
                    
                    // Ensure normal points away from origin (or viewpoint)
                    if (glm::dot(point.normal, point.position) < 0) {
                        point.normal = -point.normal;
                    }
                    
                    point.normal = glm::normalize(point.normal);
                }
            });
    }
}

PointCloud::Ptr PointCloudProcessor::transform(PointCloud::Ptr input,
                                              const glm::mat4& transformation) {
    if (!input || input->empty()) {
        return std::make_shared<PointCloud>();
    }
    
    auto output = std::make_shared<PointCloud>(*input);
    output->transform(transformation);
    return output;
}

std::vector<PointCloud::Ptr> PointCloudProcessor::euclideanClustering(PointCloud::Ptr input,
                                                                     float cluster_tolerance,
                                                                     size_t min_cluster_size,
                                                                     size_t max_cluster_size) {
    if (!input || input->empty()) {
        return {};
    }
    
    ScopedTimer timer("Euclidean Clustering");
    Logger::instance().info("Clustering {} points", input->size());
    
    std::vector<PointCloud::Ptr> clusters;
    std::vector<bool> processed(input->size(), false);
    
    // Simple region growing clustering
    for (size_t i = 0; i < input->size(); ++i) {
        if (processed[i]) continue;
        
        std::vector<size_t> cluster_indices;
        std::queue<size_t> queue;
        queue.push(i);
        processed[i] = true;
        
        while (!queue.empty()) {
            size_t current = queue.front();
            queue.pop();
            cluster_indices.push_back(current);
            
            // Find neighbors
            const auto& current_point = (*input)[current];
            
            for (size_t j = 0; j < input->size(); ++j) {
                if (!processed[j]) {
                    float dist = glm::distance(current_point.position, (*input)[j].position);
                    if (dist <= cluster_tolerance) {
                        queue.push(j);
                        processed[j] = true;
                    }
                }
            }
            
            // Check max cluster size
            if (cluster_indices.size() >= max_cluster_size) {
                break;
            }
        }
        
        // Create cluster if it meets size requirements
        if (cluster_indices.size() >= min_cluster_size && 
            cluster_indices.size() <= max_cluster_size) {
            auto cluster = std::make_shared<PointCloud>();
            cluster->reserve(cluster_indices.size());
            
            for (size_t idx : cluster_indices) {
                cluster->addPoint((*input)[idx]);
            }
            
            clusters.push_back(cluster);
        }
    }
    
    Logger::instance().info("Found {} clusters", clusters.size());
    return clusters;
}

PointCloudProcessor::Features PointCloudProcessor::computeFeatures(PointCloud::Ptr cloud) {
    Features features;
    
    if (!cloud || cloud->empty()) {
        return features;
    }
    
    // Compute centroid
    glm::vec3 centroid(0.0f);
    for (const auto& point : *cloud) {
        centroid += point.position;
    }
    centroid /= static_cast<float>(cloud->size());
    features.centroid = centroid;
    
    // Build covariance matrix
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    
    for (const auto& point : *cloud) {
        glm::vec3 diff = point.position - centroid;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                covariance(i, j) += diff[i] * diff[j];
            }
        }
    }
    covariance /= static_cast<float>(cloud->size());
    
    // Compute eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f eigenvalues = solver.eigenvalues();
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
    
    // Sort eigenvalues in descending order
    std::vector<std::pair<float, int>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
        eigen_pairs.push_back({eigenvalues[i], i});
    }
    std::sort(eigen_pairs.begin(), eigen_pairs.end(), std::greater<>());
    
    // Extract sorted eigenvalues
    features.eigenvalues = glm::vec3(eigen_pairs[0].first, 
                                    eigen_pairs[1].first, 
                                    eigen_pairs[2].first);
    
    // Extract corresponding eigenvectors
    for (int i = 0; i < 3; ++i) {
        int idx = eigen_pairs[i].second;
        for (int j = 0; j < 3; ++j) {
            features.eigenvectors[i][j] = eigenvectors(j, idx);
        }
    }
    
    // Compute shape features
    float e1 = features.eigenvalues.x;
    float e2 = features.eigenvalues.y;
    float e3 = features.eigenvalues.z;
    float sum = e1 + e2 + e3;
    
    if (sum > 0.0f) {
        features.linearity = (e1 - e2) / e1;
        features.planarity = (e2 - e3) / e1;
        features.sphericity = e3 / e1;
    }
    
    return features;
}

} // namespace CloudStream 