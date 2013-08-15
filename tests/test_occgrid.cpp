#include "occgrid.hpp"
#include <gtest/gtest.h>

///////////////////////////////////////////////////
// Test cases: Uses automatic registration of tests from gtest library
//
// The following macro runs all test in a linked test.cpp main
// return RUN_ALL_TESTS();
///////////////////////////////////////////////////

class OccupancyGrid2DTest : public ::testing::Test {
  protected:
    OccupancyGrid2D<double, int> og_;
    cv::Vec<double, 2> position_;
    double angle_;
    OccupancyGrid2DTest():
      og_(-2.5, -1.5, 0.5, 0.5, 12, 6),
      position_(0, 0),
      angle_(atan2(0.5, 2.5)) {
      }


    virtual void SetUp() {
	og_.og_ = cv::Scalar(og_.FREE);
	og_.og_.at<uint8_t>(10, 4) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(10, 1) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(11, 2) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(11, 3) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(1, 2) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(1, 3) = og_.OCCUPIED;
#ifdef DEBUG
	std::cout << og_.og_ << std::endl;
#endif
    }
};

TEST_F(OccupancyGrid2DTest, testFirstQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1), angle_, 100,
        final_pos);
    ASSERT_DOUBLE_EQ(sqrt(2.5*2.5 + 0.5*0.5), range);
    ASSERT_DOUBLE_EQ(2.5, final_pos(0));
    ASSERT_DOUBLE_EQ(0.5, final_pos(1));
}

TEST_F(OccupancyGrid2DTest, testFourthQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1), 
        - angle_, 100, final_pos);
    ASSERT_DOUBLE_EQ(range, sqrt(2.5*2.5 + 0.5*0.5));
    ASSERT_DOUBLE_EQ(final_pos(0), 2.5);
    ASSERT_DOUBLE_EQ(final_pos(1), -0.5);
}


TEST_F(OccupancyGrid2DTest, testThirdQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1),
        angle_ - bconst::pi<double>(),
	100,
	final_pos);
    double epsilon = 1e-10;
    ASSERT_NEAR(final_pos(0), -1.5, epsilon);
    ASSERT_NEAR(final_pos(1), -0.3, epsilon);
    ASSERT_NEAR(range, sqrt(1.5*1.5 + 0.3*0.3), epsilon);
}

TEST_F(OccupancyGrid2DTest, testSecondQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1),
        bconst::pi<double>() - angle_,
	100,
	final_pos);
    double eps = 1e-10;
    ASSERT_NEAR(final_pos(0), -1.5, eps);
    ASSERT_NEAR(final_pos(1), 0.3, eps);
    ASSERT_NEAR(range, sqrt(1.5*1.5 + 0.3*0.3), eps);
}

TEST_F(OccupancyGrid2DTest, testHorizontal) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1) ,
        0,
	100,
	final_pos);
    ASSERT_DOUBLE_EQ(final_pos(0), 3);
    ASSERT_DOUBLE_EQ(final_pos(1), 0);
    ASSERT_DOUBLE_EQ(range, 3);
}

TEST_F(OccupancyGrid2DTest, testNearestNeighbor) {
    cv::Vec<int, 2> neighbor;
    double dist = og_.nearest_neighbor_distance(
	cv::Vec<double, 2>(1, 0),
	6,
	neighbor);
    ASSERT_DOUBLE_EQ(dist, sqrt(1.75*1.75 + 0.75*0.75));
    ASSERT_EQ(neighbor(0), 10);
    ASSERT_EQ(neighbor(1), 4);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
