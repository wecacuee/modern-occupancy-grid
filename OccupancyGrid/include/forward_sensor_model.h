#ifndef FORWARD_SENSOR_MODEL_H
#define FORWARD_SENSOR_MODEL_H

#include <opencv2/opencv.hpp>
#include <occgrid.hpp>

class Observation2D {
  public:
    double px, py, ptheta, range;

    Observation2D() {}

    Observation2D(double px, 
        double py,
        double ptheta,
        double range) : px(px),py(py), ptheta(ptheta), range(range) {}
};

double log_odds_observation_given_map_and_pose(
    const Observation2D& observation,
    OccupancyGrid2D<double, int>& map);

double log_odds_observation_given_map_and_all_poses(
    const std::vector<Observation2D>& observations,
    OccupancyGrid2D<double, int>& map);
#endif // FORWARD_SENSOR_MODEL_H
