#include <gtest/gtest.h>
#include "core/PointCloud.h"
#include <glm/gtc/epsilon.hpp>

using namespace CloudStream;

class PointCloudTest : public ::testing::Test {
protected:
    void SetUp() override {
        cloud = std::make_shared<PointCloud>();
    }
    
    PointCloud::Ptr cloud;
};

TEST_F(PointCloudTest, DefaultConstruction) {
    EXPECT_TRUE(cloud->empty());
    EXPECT_EQ(cloud->size(), 0);
}

TEST_F(PointCloudTest, AddPoint) {
    Point p;
    p.position = glm::vec3(1.0f, 2.0f, 3.0f);
    p.color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    
    cloud->addPoint(p);
    
    EXPECT_FALSE(cloud->empty());
    EXPECT_EQ(cloud->size(), 1);
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[0].position, p.position, 0.001f)));
}

TEST_F(PointCloudTest, AddMultiplePoints) {
    std::vector<Point> points;
    for (int i = 0; i < 100; ++i) {
        Point p;
        p.position = glm::vec3(i, i * 2, i * 3);
        points.push_back(p);
    }
    
    cloud->addPoints(points);
    
    EXPECT_EQ(cloud->size(), 100);
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[50].position, glm::vec3(50, 100, 150), 0.001f)));
}

TEST_F(PointCloudTest, BoundingBox) {
    std::vector<Point> points = {
        Point(glm::vec3(-1, -2, -3)),
        Point(glm::vec3(4, 5, 6)),
        Point(glm::vec3(0, 0, 0))
    };
    
    cloud->addPoints(points);
    
    auto bbox = cloud->getBoundingBox();
    EXPECT_TRUE(glm::all(glm::epsilonEqual(bbox.min, glm::vec3(-1, -2, -3), 0.001f)));
    EXPECT_TRUE(glm::all(glm::epsilonEqual(bbox.max, glm::vec3(4, 5, 6), 0.001f)));
    EXPECT_TRUE(glm::all(glm::epsilonEqual(bbox.center(), glm::vec3(1.5f, 1.5f, 1.5f), 0.001f)));
}

TEST_F(PointCloudTest, Transform) {
    cloud->addPoint(Point(glm::vec3(1, 0, 0)));
    cloud->addPoint(Point(glm::vec3(0, 1, 0)));
    
    // Translate by (1, 2, 3)
    cloud->translate(glm::vec3(1, 2, 3));
    
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[0].position, glm::vec3(2, 2, 3), 0.001f)));
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[1].position, glm::vec3(1, 3, 3), 0.001f)));
}

TEST_F(PointCloudTest, Scale) {
    cloud->addPoint(Point(glm::vec3(2, 4, 6)));
    
    cloud->scale(0.5f);
    
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[0].position, glm::vec3(1, 2, 3), 0.001f)));
}

TEST_F(PointCloudTest, RemoveNaN) {
    cloud->addPoint(Point(glm::vec3(1, 2, 3)));
    cloud->addPoint(Point(glm::vec3(NAN, 0, 0)));
    cloud->addPoint(Point(glm::vec3(0, INFINITY, 0)));
    cloud->addPoint(Point(glm::vec3(4, 5, 6)));
    
    EXPECT_EQ(cloud->size(), 4);
    
    cloud->removeNaN();
    
    EXPECT_EQ(cloud->size(), 2);
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[0].position, glm::vec3(1, 2, 3), 0.001f)));
    EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[1].position, glm::vec3(4, 5, 6), 0.001f)));
}

TEST_F(PointCloudTest, Downsample) {
    // Create a grid of points
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            for (int z = 0; z < 10; ++z) {
                cloud->addPoint(Point(glm::vec3(x * 0.1f, y * 0.1f, z * 0.1f)));
            }
        }
    }
    
    EXPECT_EQ(cloud->size(), 1000);
    
    // Downsample with voxel size 0.3
    cloud->downsample(0.3f);
    
    // Should have significantly fewer points
    EXPECT_LT(cloud->size(), 100);
    EXPECT_GT(cloud->size(), 10);
}

TEST_F(PointCloudTest, Serialization) {
    // Add some test points
    for (int i = 0; i < 10; ++i) {
        Point p;
        p.position = glm::vec3(i, i * 2, i * 3);
        p.normal = glm::vec3(0, 1, 0);
        p.color = glm::vec4(1, 0, 0, 1);
        p.intensity = 0.5f;
        cloud->addPoint(p);
    }
    
    // Serialize
    auto data = cloud->serialize();
    EXPECT_GT(data.size(), 0);
    
    // Deserialize into new cloud
    auto cloud2 = std::make_shared<PointCloud>();
    cloud2->deserialize(data);
    
    // Compare
    EXPECT_EQ(cloud->size(), cloud2->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[i].position, (*cloud2)[i].position, 0.001f)));
        EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[i].normal, (*cloud2)[i].normal, 0.001f)));
        EXPECT_TRUE(glm::all(glm::epsilonEqual((*cloud)[i].color, (*cloud2)[i].color, 0.001f)));
        EXPECT_FLOAT_EQ((*cloud)[i].intensity, (*cloud2)[i].intensity);
    }
}

TEST_F(PointCloudTest, Statistics) {
    // Create points in a known configuration
    cloud->addPoint(Point(glm::vec3(0, 0, 0)));
    cloud->addPoint(Point(glm::vec3(2, 0, 0)));
    cloud->addPoint(Point(glm::vec3(0, 2, 0)));
    cloud->addPoint(Point(glm::vec3(0, 0, 2)));
    
    auto stats = cloud->computeStatistics();
    
    EXPECT_EQ(stats.point_count, 4);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(stats.centroid, glm::vec3(0.5f, 0.5f, 0.5f), 0.001f)));
    EXPECT_GT(stats.average_density, 0.0f);
}

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 