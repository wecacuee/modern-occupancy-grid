// c
#include <cmath>

// std
#include <iostream>

// 3rd party
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

// local
#include <occgrid.hpp>

// me
#include <forward_sensor_model.h>

namespace bconst = boost::math::constants;
namespace brand = boost::random;

#undef DEBUGDRAW

double log_gaussian1d(double x, double mu, double sigma) {
    return (-0.5 * (x - mu)*(x - mu) / (sigma*sigma)) + log(sigma * bconst::root_two_pi<double>());
}

double gaussian1d(double x, double mu, double sigma) {
    // ignore the scaling factor
    return exp(-0.5 * (x - mu)*(x - mu) / (sigma*sigma));//  / (sigma * bconst::root_two_pi<double>());
}

//double probability_single_observation_given_map_and_pose(
double log_odds_observation_given_map_and_pose(
    const Observation2D& observation,
    OccupancyGrid2D<double, int>& map,
    double& expected_range) 
{
    double total_angle = observation.ptheta;
    cv::Vec2d direction(cos(total_angle), sin(total_angle));
    cv::Vec2d position(observation.px, observation.py);
    // if (std::isnan(direction(0))) {
    //     printf("Robot Angle : %f, angle of obs: %f\n", robot_angle, angle_of_observation);
    //     throw std::logic_error("direction(0) is nan");
    // }
    //
    assert(! std::isnan(direction(1)));
    assert(! std::isnan(direction(0)));
    cv::Vec2d final_pos;
    expected_range = map.ray_trace(observation.px, observation.py,
        observation.ptheta, LASER_MAX_RANGE, final_pos);
    //if (expected_range == LASER_MAX_RANGE) {
        // laser didn't strike any wall
        // but that doesn't matter, because our input observations also
        // contain saturated readings instead of providing some special value.
    //}
    double sigma = NOISE_VARIANCE * expected_range; // noise increases with distance
#ifdef DEBUG
      printf("Sigma:%f\n", sigma);
      printf("Expected range:%f\n", expected_range);
      printf("Observed range:%f\n", observation.range);
#endif
    double gaussian_lodds = log_gaussian1d(observation.range, expected_range, sigma);
    return gaussian_lodds;
}

class ForwardSensorModelTest : public ::testing::Test {
  protected:
    OccupancyGrid2D<double, int> map;
    boost::mt19937 gen;
    double range_observation; 
    boost::normal_distribution<> norm_dist;;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > norm_rand;;

    ForwardSensorModelTest():
      map(-2.5, -1.5, 0.5, 0.5, 12, 6),
      gen(),
      range_observation(sqrt(2.5*2.5 + 0.5*0.5)),
      norm_dist(0, NOISE_VARIANCE * range_observation),
      norm_rand(gen, norm_dist)
    { }

    virtual void SetUp() {
        map.og_ = cv::Scalar(map.FREE);
        map.og_.at<uint8_t>(10, 4) = map.OCCUPIED;
        map.og_.at<uint8_t>(10, 1) = map.OCCUPIED;
        map.og_.at<uint8_t>(11, 2) = map.OCCUPIED;
        map.og_.at<uint8_t>(11, 3) = map.OCCUPIED;
        map.og_.at<uint8_t>(1, 2) = map.OCCUPIED;
        map.og_.at<uint8_t>(1, 3) = map.OCCUPIED;
        range_observation = sqrt(2.5*2.5 + 0.5*0.5);
    }
};

TEST_F(ForwardSensorModelTest, test1) {
    double noise = norm_rand();
    range_observation += noise;
    double robot_angle = atan2(0.5, 2.5);
    Observation2D observation(0, 0, robot_angle, range_observation);
    double exp_range;
    double lodds =
      log_odds_observation_given_map_and_pose(
          observation,
          map,
          exp_range);

    ASSERT_NEAR(
        log_gaussian1d(noise, 0, NOISE_VARIANCE * range_observation),
        lodds, 
        0.002);
}

//double probability_observation_given_map_and_all_poses(
double log_odds_observation_given_map_and_all_poses(
    const std::vector<Observation2D>& observations,
    OccupancyGrid2D<double, int>& map)
{
    std::vector<double> lodds_vector;
    lodds_vector.reserve(observations.size());
#ifdef DEBUGDRAW
    int scan_count = 0;
    double last_px = observations[0].px;
    double last_px = observations[1].py;
    std::vector<double> angles;
    std::vector<double> ranges;
    std::vector<double> exp_ranges;
#endif
    for (std::vector<Observation2D>::const_iterator it = observations.begin();
        it != observations.end();
        ++it) {
        double expr;
        lodds_vector.push_back(
            log_odds_observation_given_map_and_pose(*it, map, expr));
#ifdef DEBUGDRAW
        // when position of the robot changes
        // draw lasers observations collected till now.
        if (last_px != (*it).px && last_py != (*it).py) {
            cv::Vec2d position(last_px, last_py);
            last_px = (*it).px;
            last_py = (*it).py;
            map.draw_and_show_lasers(position, 0, &angles[0],
                &exp_ranges[0],
                &ranges[0],
                scan_count);
            // reset
            scan_count = 0;
            angles.clear();
            ranges.clear();
            exp_ranges.clear();
        }
        scan_count++;
        angles.push_back((*it).ptheta);
        ranges.push_back((*it).range);
        exp_ranges.push_back(expr);
#endif
    }

    double lodds = 0;
    for (size_t i = 0; i < observations.size(); i ++) {
        lodds += lodds_vector[i];
    }
    return lodds;
}
