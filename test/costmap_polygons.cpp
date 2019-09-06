#include <random>
#include <memory>
#include <gtest/gtest.h>

#include <costmap_converter/costmap_to_polygons.h>

// make things accesible in the test
class CostmapToPolygons : public costmap_converter::CostmapToPolygonsDBSMCCH
{
  public:
    const std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>& points() const {return occupied_cells_;}
    costmap_converter::CostmapToPolygonsDBSMCCH::Parameters& parameters() {return parameter_;}
    using costmap_converter::CostmapToPolygonsDBSMCCH::addPoint;
    using costmap_converter::CostmapToPolygonsDBSMCCH::regionQuery;
    using costmap_converter::CostmapToPolygonsDBSMCCH::dbScan;
    using costmap_converter::CostmapToPolygonsDBSMCCH::convexHull2;
};

class CostmapToPolygonsDBSMCCHTest : public ::testing::Test
{
  protected:
    void SetUp() override {
      // parameters
      costmap_to_polygons.parameters().max_distance_ = 0.5;
      costmap_to_polygons.parameters().max_pts_ = 100;
      costmap_to_polygons.parameters().min_pts_ = 2;
      costmap_to_polygons.parameters().min_keypoint_separation_ = 0.1;

      costmap.reset(new costmap_2d::Costmap2D(100, 100, 0.1, -5., -5.));
      costmap_to_polygons.setCostmap2D(costmap.get());

      std::random_device rand_dev;
      std::mt19937 generator(rand_dev());
      std::uniform_real_distribution<> random_angle(-M_PI, M_PI);
      std::uniform_real_distribution<> random_dist(0.,   costmap_to_polygons.parameters().max_distance_);

      costmap_to_polygons.addPoint(0., 0.);
      costmap_to_polygons.addPoint(1.3, 1.3);

      // adding random points
      double center_x = costmap_to_polygons.points()[0].x;
      double center_y = costmap_to_polygons.points()[0].y;
      for (int i = 0; i < costmap_to_polygons.parameters().max_pts_ - 1; ++i)
      {
        double alpha = random_angle(generator);
        double dist  = random_dist(generator);
        double wx = center_x + std::cos(alpha) * dist;
        double wy = center_y + std::sin(alpha) * dist;
        costmap_to_polygons.addPoint(wx, wy);
      }

      // some noisy points not belonging to a cluster
      costmap_to_polygons.addPoint(-1, -1);
      costmap_to_polygons.addPoint(-2, -2);
      
      // adding random points
      center_x = costmap_to_polygons.points()[1].x;
      center_y = costmap_to_polygons.points()[1].y;
      for (int i = 0; i < costmap_to_polygons.parameters().max_pts_/2; ++i)
      {
        double alpha = random_angle(generator);
        double dist  = random_dist(generator);
        double wx = center_x + std::cos(alpha) * dist;
        double wy = center_y + std::sin(alpha) * dist;
        costmap_to_polygons.addPoint(wx, wy);
      }
    }

    void regionQueryTrivial(int curr_index, std::vector<int>& neighbor_indices)
    {
      neighbor_indices.clear();
      const auto& query_point = costmap_to_polygons.points()[curr_index];
      for (int i = 0; i < int(costmap_to_polygons.points().size()); ++i)
      {
        if (i == curr_index)
          continue;
        const auto& kp = costmap_to_polygons.points()[i];
        double dx = query_point.x - kp.x;
        double dy = query_point.y - kp.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < costmap_to_polygons.parameters().max_distance_)
          neighbor_indices.push_back(i);
      }
    }

    CostmapToPolygons costmap_to_polygons;
    std::shared_ptr<costmap_2d::Costmap2D> costmap;
};

TEST_F(CostmapToPolygonsDBSMCCHTest, regionQuery)
{
  std::vector<int> neighbors, neighbors_trivial;
  costmap_to_polygons.regionQuery(0, neighbors);
  regionQueryTrivial(0, neighbors_trivial);
  ASSERT_EQ(costmap_to_polygons.parameters().max_pts_ - 1, int(neighbors.size()));
  ASSERT_EQ(neighbors_trivial.size(), neighbors.size());
  std::sort(neighbors.begin(), neighbors.end());
  ASSERT_EQ(neighbors_trivial, neighbors);

  costmap_to_polygons.regionQuery(1, neighbors);
  regionQueryTrivial(1, neighbors_trivial);
  ASSERT_EQ(costmap_to_polygons.parameters().max_pts_/2, int(neighbors.size()));
  ASSERT_EQ(neighbors_trivial.size(), neighbors.size());
  std::sort(neighbors.begin(), neighbors.end());
  ASSERT_EQ(neighbors_trivial, neighbors);
}

TEST_F(CostmapToPolygonsDBSMCCHTest, dbScan)
{
  std::vector< std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> > clusters;
  costmap_to_polygons.dbScan(clusters);
  
  ASSERT_EQ(3, clusters.size());
  ASSERT_EQ(2, clusters[0].size()); // noisy points not belonging to a cluster
  ASSERT_EQ(costmap_to_polygons.parameters().max_pts_, clusters[1].size()); // first cluster at (0,0)
  ASSERT_EQ(costmap_to_polygons.parameters().max_pts_/2 + 1, clusters[2].size()); // second cluster at (1,1)
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "CostmapConverterTester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}