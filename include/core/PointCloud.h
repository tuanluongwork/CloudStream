#pragma once

#include <vector>
#include <memory>
#include <array>
#include <cstdint>
#include <algorithm>
#include <execution>
#include <mutex>
#include <glm/glm.hpp>

namespace CloudStream {

/**
 * @brief Represents a single point in 3D space with color and normal information
 */
struct Point {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec4 color{1.0f};
    float intensity{1.0f};
    
    Point() = default;
    Point(const glm::vec3& pos, const glm::vec4& col = glm::vec4(1.0f))
        : position(pos), color(col) {}
};

/**
 * @brief Container for point cloud data with efficient memory management
 */
class PointCloud {
public:
    using PointContainer = std::vector<Point>;
    using Ptr = std::shared_ptr<PointCloud>;
    using ConstPtr = std::shared_ptr<const PointCloud>;
    
    PointCloud() = default;
    explicit PointCloud(size_t reserve_size);
    PointCloud(const PointCloud&) = default;
    PointCloud(PointCloud&&) noexcept = default;
    PointCloud& operator=(const PointCloud&) = default;
    PointCloud& operator=(PointCloud&&) noexcept = default;
    
    // Point access
    void addPoint(const Point& point);
    void addPoints(const std::vector<Point>& points);
    Point& operator[](size_t index);
    const Point& operator[](size_t index) const;
    
    // Container operations
    size_t size() const noexcept { return points_.size(); }
    bool empty() const noexcept { return points_.empty(); }
    void clear() noexcept;
    void reserve(size_t size);
    void resize(size_t size);
    
    // Iterators
    auto begin() noexcept { return points_.begin(); }
    auto end() noexcept { return points_.end(); }
    auto begin() const noexcept { return points_.begin(); }
    auto end() const noexcept { return points_.end(); }
    
    // Bounding box
    struct BoundingBox {
        glm::vec3 min{std::numeric_limits<float>::max()};
        glm::vec3 max{std::numeric_limits<float>::lowest()};
        
        glm::vec3 center() const { return (min + max) * 0.5f; }
        glm::vec3 size() const { return max - min; }
        float diagonal() const { return glm::length(size()); }
    };
    
    const BoundingBox& getBoundingBox() const;
    void updateBoundingBox();
    
    // Transformations
    void transform(const glm::mat4& matrix);
    void translate(const glm::vec3& translation);
    void rotate(float angle, const glm::vec3& axis);
    void scale(float factor);
    void scale(const glm::vec3& factors);
    
    // Filtering
    void removeNaN();
    void downsample(float voxel_size);
    void statisticalOutlierRemoval(int k_neighbors, float std_dev_mul_thresh);
    
    // Serialization
    std::vector<uint8_t> serialize() const;
    void deserialize(const std::vector<uint8_t>& data);
    
    // Statistics
    struct Statistics {
        size_t point_count{0};
        glm::vec3 centroid{0.0f};
        float average_density{0.0f};
        BoundingBox bounds;
    };
    
    Statistics computeStatistics() const;
    
private:
    PointContainer points_;
    mutable BoundingBox bounding_box_;
    mutable bool bbox_dirty_{true};
    mutable std::mutex mutex_;
};

} // namespace CloudStream 