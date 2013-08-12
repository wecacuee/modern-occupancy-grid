#ifndef FORWARD_SENSOR_MODEL_H
#define FORWARD_SENSOR_MODEL_H

#include <opencv2/opencv.hpp>
#include <occgrid.hpp>

// from stage/examples/ctrl/lasernoise.cc
#define NOISE_VARIANCE 0.05 // ratio to range

// from stage/libstage/model_laser.cc
#define LASER_MAX_RANGE 8.0 // in meters

// Abondoned because requires refactoring of ray_trace method to accept a
// function pointer instead of overriding a class.
//
// /**
//  * Keeps a mapping from cells to factors that affect those cells
//  */
// class FactorGraph {
//   public:
//     cv::Mat_<std::vector< Observation2D > > cell2observation_;
//     FactorGraph(
//         const std::vector< Observation2D >& observations, 
//         const OccupancyGrid2D<double, int>& map) 
//     {
//       typedef std::vector< Observation2D >::iterator it_type;
//       for (it_type it = observations.begin(); it != observations.end(); it++) 
//       {
//         map.ray_trace(it->px, it->py, it->ptheta,    
//       }
//     }
// };

double log_odds_observation_given_map_and_pose(
    const Observation2D& observation,
    OccupancyGrid2D<double, int>& map);

double log_odds_observation_given_map_and_all_poses(
    const std::vector<Observation2D>& observations,
    OccupancyGrid2D<double, int>& map);

double log_odds_occupied_vs_free(
    const std::vector<Observation2D>& observations,
    OccupancyGrid2D<double, int>& map,
    int k);

#endif // FORWARD_SENSOR_MODEL_H
