#include "core/PointCloud.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <unordered_map>

namespace CloudStream {

PointCloud::PointCloud(size_t reserve_size) {
    points_.reserve(reserve_size);
}

void PointCloud::addPoint(const Point& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.push_back(point);
    bbox_dirty_ = true;
}

void PointCloud::addPoints(const std::vector<Point>& points) {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.insert(points_.end(), points.begin(), points.end());
    bbox_dirty_ = true;
}

Point& PointCloud::operator[](size_t index) {
    bbox_dirty_ = true;
    return points_[index];
}

const Point& PointCloud::operator[](size_t index) const {
    return points_[index];
}

void PointCloud::clear() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.clear();
    bbox_dirty_ = true;
}

void PointCloud::reserve(size_t size) {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.reserve(size);
}

void PointCloud::resize(size_t size) {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.resize(size);
    bbox_dirty_ = true;
}

const PointCloud::BoundingBox& PointCloud::getBoundingBox() const {
    if (bbox_dirty_) {
        const_cast<PointCloud*>(this)->updateBoundingBox();
    }
    return bounding_box_;
}

void PointCloud::updateBoundingBox() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (points_.empty()) {
        bounding_box_ = BoundingBox{};
        bbox_dirty_ = false;
        return;
    }
    
    bounding_box_.min = glm::vec3(std::numeric_limits<float>::max());
    bounding_box_.max = glm::vec3(std::numeric_limits<float>::lowest());
    
    // Parallel reduction for better performance on large clouds
    std::for_each(std::execution::par_unseq, points_.begin(), points_.end(),
        [this](const Point& p) {
            // Thread-safe min/max using atomic operations would be ideal here
            // For now, we'll compute serially after parallel pass
        });
    
    // Serial pass for now (can be optimized with atomic operations)
    for (const auto& point : points_) {
        bounding_box_.min = glm::min(bounding_box_.min, point.position);
        bounding_box_.max = glm::max(bounding_box_.max, point.position);
    }
    
    bbox_dirty_ = false;
}

void PointCloud::transform(const glm::mat4& matrix) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    glm::mat3 normal_matrix = glm::transpose(glm::inverse(glm::mat3(matrix)));
    
    std::for_each(std::execution::par_unseq, points_.begin(), points_.end(),
        [&matrix, &normal_matrix](Point& p) {
            p.position = glm::vec3(matrix * glm::vec4(p.position, 1.0f));
            p.normal = glm::normalize(normal_matrix * p.normal);
        });
    
    bbox_dirty_ = true;
}

void PointCloud::translate(const glm::vec3& translation) {
    transform(glm::translate(glm::mat4(1.0f), translation));
}

void PointCloud::rotate(float angle, const glm::vec3& axis) {
    transform(glm::rotate(glm::mat4(1.0f), angle, axis));
}

void PointCloud::scale(float factor) {
    scale(glm::vec3(factor));
}

void PointCloud::scale(const glm::vec3& factors) {
    transform(glm::scale(glm::mat4(1.0f), factors));
}

void PointCloud::removeNaN() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto new_end = std::remove_if(points_.begin(), points_.end(),
        [](const Point& p) {
            return std::isnan(p.position.x) || std::isnan(p.position.y) || 
                   std::isnan(p.position.z) || std::isinf(p.position.x) ||
                   std::isinf(p.position.y) || std::isinf(p.position.z);
        });
    
    points_.erase(new_end, points_.end());
    bbox_dirty_ = true;
}

void PointCloud::downsample(float voxel_size) {
    if (voxel_size <= 0.0f || points_.empty()) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Compute voxel grid bounds
    updateBoundingBox();
    glm::vec3 min_bound = bounding_box_.min;
    glm::vec3 max_bound = bounding_box_.max;
    
    // Voxel grid dimensions
    glm::ivec3 grid_size(
        static_cast<int>((max_bound.x - min_bound.x) / voxel_size) + 1,
        static_cast<int>((max_bound.y - min_bound.y) / voxel_size) + 1,
        static_cast<int>((max_bound.z - min_bound.z) / voxel_size) + 1
    );
    
    // Hash map for voxel occupancy
    std::unordered_map<size_t, std::vector<size_t>> voxel_map;
    
    // Assign points to voxels
    for (size_t i = 0; i < points_.size(); ++i) {
        const Point& p = points_[i];
        glm::ivec3 voxel_idx(
            static_cast<int>((p.position.x - min_bound.x) / voxel_size),
            static_cast<int>((p.position.y - min_bound.y) / voxel_size),
            static_cast<int>((p.position.z - min_bound.z) / voxel_size)
        );
        
        size_t hash = voxel_idx.x + voxel_idx.y * grid_size.x + 
                     voxel_idx.z * grid_size.x * grid_size.y;
        voxel_map[hash].push_back(i);
    }
    
    // Average points in each voxel
    std::vector<Point> downsampled;
    downsampled.reserve(voxel_map.size());
    
    for (const auto& [hash, indices] : voxel_map) {
        Point avg_point;
        glm::vec3 avg_pos(0.0f);
        glm::vec3 avg_normal(0.0f);
        glm::vec4 avg_color(0.0f);
        float avg_intensity = 0.0f;
        
        for (size_t idx : indices) {
            const Point& p = points_[idx];
            avg_pos += p.position;
            avg_normal += p.normal;
            avg_color += p.color;
            avg_intensity += p.intensity;
        }
        
        float inv_count = 1.0f / indices.size();
        avg_point.position = avg_pos * inv_count;
        avg_point.normal = glm::normalize(avg_normal);
        avg_point.color = avg_color * inv_count;
        avg_point.intensity = avg_intensity * inv_count;
        
        downsampled.push_back(avg_point);
    }
    
    points_ = std::move(downsampled);
    bbox_dirty_ = true;
}

void PointCloud::statisticalOutlierRemoval(int k_neighbors, float std_dev_mul_thresh) {
    if (points_.size() < static_cast<size_t>(k_neighbors)) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // For each point, compute mean distance to k nearest neighbors
    std::vector<float> mean_distances(points_.size());
    
    // Simple brute force approach for neighbor search
    // In production, use KD-tree or octree for better performance
    std::for_each(std::execution::par_unseq, 
                  points_.begin(), points_.end(),
                  [this, k_neighbors, &mean_distances](const Point& point) {
        size_t idx = &point - &points_[0];
        std::vector<float> distances;
        distances.reserve(points_.size() - 1);
        
        // Compute distances to all other points
        for (size_t j = 0; j < points_.size(); ++j) {
            if (j != idx) {
                float dist = glm::distance(point.position, points_[j].position);
                distances.push_back(dist);
            }
        }
        
        // Sort and take k nearest
        std::partial_sort(distances.begin(), 
                         distances.begin() + std::min(k_neighbors, static_cast<int>(distances.size())),
                         distances.end());
        
        // Compute mean distance
        float sum = 0.0f;
        int count = std::min(k_neighbors, static_cast<int>(distances.size()));
        for (int i = 0; i < count; ++i) {
            sum += distances[i];
        }
        mean_distances[idx] = sum / count;
    });
    
    // Compute global mean and standard deviation
    float global_mean = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0f) 
                       / mean_distances.size();
    
    float variance = 0.0f;
    for (float dist : mean_distances) {
        float diff = dist - global_mean;
        variance += diff * diff;
    }
    variance /= mean_distances.size();
    float std_dev = std::sqrt(variance);
    
    // Filter points based on threshold
    float threshold = global_mean + std_dev_mul_thresh * std_dev;
    
    std::vector<Point> filtered;
    filtered.reserve(points_.size());
    
    for (size_t i = 0; i < points_.size(); ++i) {
        if (mean_distances[i] <= threshold) {
            filtered.push_back(points_[i]);
        }
    }
    
    points_ = std::move(filtered);
    bbox_dirty_ = true;
}

std::vector<uint8_t> PointCloud::serialize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Header: magic number + version + point count
    const uint32_t magic = 0x50434C44; // "PCLD"
    const uint32_t version = 1;
    const uint32_t point_count = static_cast<uint32_t>(points_.size());
    
    size_t total_size = sizeof(magic) + sizeof(version) + sizeof(point_count) +
                       point_count * sizeof(Point);
    
    std::vector<uint8_t> buffer;
    buffer.reserve(total_size);
    
    // Write header
    auto append_bytes = [&buffer](const void* data, size_t size) {
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        buffer.insert(buffer.end(), bytes, bytes + size);
    };
    
    append_bytes(&magic, sizeof(magic));
    append_bytes(&version, sizeof(version));
    append_bytes(&point_count, sizeof(point_count));
    
    // Write points
    append_bytes(points_.data(), point_count * sizeof(Point));
    
    return buffer;
}

void PointCloud::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < 12) {
        throw std::runtime_error("Invalid point cloud data: too small");
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    size_t offset = 0;
    auto read_bytes = [&data, &offset](void* dest, size_t size) {
        if (offset + size > data.size()) {
            throw std::runtime_error("Invalid point cloud data: unexpected end");
        }
        std::memcpy(dest, data.data() + offset, size);
        offset += size;
    };
    
    // Read header
    uint32_t magic, version, point_count;
    read_bytes(&magic, sizeof(magic));
    read_bytes(&version, sizeof(version));
    read_bytes(&point_count, sizeof(point_count));
    
    if (magic != 0x50434C44) {
        throw std::runtime_error("Invalid point cloud data: wrong magic number");
    }
    
    if (version != 1) {
        throw std::runtime_error("Unsupported point cloud version");
    }
    
    // Read points
    points_.resize(point_count);
    read_bytes(points_.data(), point_count * sizeof(Point));
    
    bbox_dirty_ = true;
}

PointCloud::Statistics PointCloud::computeStatistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Statistics stats;
    stats.point_count = points_.size();
    
    if (points_.empty()) {
        return stats;
    }
    
    // Compute centroid
    glm::vec3 sum(0.0f);
    for (const auto& p : points_) {
        sum += p.position;
    }
    stats.centroid = sum / static_cast<float>(points_.size());
    
    // Get bounding box
    stats.bounds = getBoundingBox();
    
    // Estimate average density (points per unit volume)
    glm::vec3 size = stats.bounds.size();
    float volume = size.x * size.y * size.z;
    if (volume > 0.0f) {
        stats.average_density = static_cast<float>(points_.size()) / volume;
    }
    
    return stats;
}

} // namespace CloudStream 