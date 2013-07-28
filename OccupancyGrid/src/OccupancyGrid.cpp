/**
 *      @file OccupancyGrid.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "OccupancyGrid.h"
#include <cassert>

using namespace std;
using namespace gtsam;

/*****************************************************************************/
OccupancyGrid::OccupancyGrid(double width, double height, double resolution) :
  width_( width / resolution),
  height_(height / resolution),
  res_(resolution),
  heat_map_(cellCount(), 1),
  cell2factors_(cellCount())
{

}

/*****************************************************************************/
LaserFactor::Occupancy OccupancyGrid::emptyOccupancy() const {
  LaserFactor::Occupancy occupancy; //mapping from Index to value (0 or 1)
  for (size_t i = 0; i < cellCount(); i++)
    occupancy.insert(pair<Index, size_t>((Index) i, 0));

  return occupancy;
}

/*****************************************************************************/
void OccupancyGrid::addPrior(Index cell, double prior) {
  size_t numStates = 2;
  DiscreteKey key(cell, numStates);

  //add a factor
  vector<double> table(2);
  table[0] = 1 - prior;
  table[1] = prior;
  add(key, table);
}

void OccupancyGrid::rayTrace(const gtsam::Pose2 &pose, const double range,
    vector<gtsam::Index>& cells,
    gtsam::Index& key) {
  // ray trace from pose to range t//a >= 1 accept new state find all cells the laser passes through
  double x = pose.x(); //start position of the laser
  double y = pose.y();
  double step = res_; //amount to step in each iteration of laser traversal


  // TODO: use integer addition based ray tracing.
  // traverse laser to find all the cells
  for (double i = 0; i < range; i += step) {
    //get point on laser
    x = pose.x() + i * cos(pose.theta());
    y = pose.y() + i * sin(pose.theta());

    // Get the key of the cell that holds point (x,y)
    key = keyLookup(x, y);

    // add cell to list of cells if it is new
    if (((i == 0 // first element in cells
            || (key != cells[cells.size() - 1])) // we just added it last time
          && (key < (cellCount() - 1))))  // must be inside occupancy grid
    {
      cells.push_back(key);
    }
  }
}

void
OccupancyGrid::addLaser(const Pose2 &pose, double range)
{
  vector<Index> cells; // list of keys of cells hit by the laser
  Index key;
  rayTrace(pose, range, cells, key);

  // last cell hit by laser has higher probability to be flipped
  if (key < (cellCount() - 1))
    heat_map_[key] = 4;

  // add a factor that connects all those cells (if there are any)
  if (cells.size() > 0) {
    // also, store some book-keeping info in this class
    pose_.push_back(pose);
    range_.push_back(range);
    push_back(make_laser(cells));
  }
}

/*****************************************************************************/
Index OccupancyGrid::keyLookup(double x, double y) const {
  //move (x,y) to the nearest resolution
  x *= (1.0 / res_);
  y *= (1.0 / res_);

  //round to nearest integer
  x = (double) ((int) x);
  y = (double) ((int) y);

  //determine index
  x += width_ / 2;
  y = height_ / 2 - y;

  //bounds checking
  size_t index = y * width_ + x;
  return (index >= width_ * height_) ? -1 : index;
}

/*****************************************************************************/
double OccupancyGrid::operator()(const LaserFactor::Occupancy &occupancy) const {
  double value = 0;
  // loop over all laser factors in the graph
  typedef typename std::vector<
    boost::shared_ptr<
    gtsam::DiscreteFactor> >::const_iterator const_iterator;
  for (const_iterator f(factors_.begin()); f != factors_.end(); ++f)
    value += (**f)(occupancy);
  return value;
}

/*****************************************************************************/
double OccupancyGrid::computeDelta(LaserFactor::Occupancy &occupancy,
    const Index &cellidx, size_t newValue) const
{
  vector< gtsam::Index > recomputefactors = cell2factors_[cellidx];
  double oldEnergy = 0;
  for (vector< gtsam::Index >::iterator it = recomputefactors.begin();
      it != recomputefactors.end(); it++)
    oldEnergy += laserFactorValue(*it, occupancy);

  // Ensures rollback to original value of occupancy grid
  class TempGrid {
    public:
      LaserFactor::Occupancy &occ_;
      const IndexType &cellidx_;
      const ValueType oldValue_;
      TempGrid(LaserFactor::Occupancy& occ, const IndexType& cellidx,
          const ValueType &newValue
          ) : occ_(occ),
      cellidx_(cellidx),
      oldValue_(occ.at(cellidx)) {
        occ_[cellidx_] = newValue;
      }

      ~TempGrid() {
        occ_[cellidx_] = oldValue_;
      }
  } newocc( occupancy , cellidx, newValue);

  double newEnergy = 0;
  for (vector< gtsam::Index >::iterator it = recomputefactors.begin();
      it != recomputefactors.end(); it++)
    newEnergy += laserFactorValue(*it, occupancy);
  return newEnergy - oldEnergy;
}

/*****************************************************************************/
void OccupancyGrid::saveLaser(const char *fname) const {
  FILE *fptr = fopen(fname, "w");
  LaserFactor::Occupancy occupancy;
  for (Index i = 0; i < pose_.size(); i++) {
    fprintf(fptr, "%lf %lf %lf %lf\n", pose_[i].x(), pose_[i].y(),
        pose_[i].theta(), range_[i]);
  }
  fclose(fptr);
}

/*****************************************************************************/
void OccupancyGrid::saveHeatMap(const char *fname) const {
  FILE *fptr = fopen(fname, "wb");
  fprintf(fptr, "P3 %d %d 255\n", (int) width_, (int) height_);

  //unsigned int red, green, blue;
  for (size_t it = 0; it < cellCount(); it++) {
    if (heat_map_[it] == 4)
      fprintf(fptr, "255 0 0\n");
    else
      fprintf(fptr, "0 0 255\n");
  }
  fclose(fptr);

}

///* ************************************************************************* */
//TEST_UNSAFE( OccupancyGrid, Test1) {
//	//Build a small grid and test optimization
//
//	//Build small grid
//	double width 		=	20; 		//meters
//	double height 		= 	20; 		//meters
//	double resolution 	= 	0.2; 	//meters
//	OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle
//
//	//Add measurements
////	Pose2 pose(0,0,0);
////	double range = 4.499765;
////
////	occupancyGrid.addPrior(0, 0.7);
////	EXPECT_LONGS_EQUAL(1, occupancyGrid.size());
////
////	occupancyGrid.addLaser(pose, range);
////	EXPECT_LONGS_EQUAL(2, occupancyGrid.size());
//
//	//add lasers
//	int n_frames = 1;
//	int n_lasers_per_frame = 640;
//	char laser_list_file[1000];
//
//
//	for(int i = 0; i < n_frames; i++){
//		sprintf(laser_list_file, "/home/brian/Desktop/research/user/bpeasle/code/KinectInterface/Data/ScanLinesAsLasers/KinectRecording9/laser_list%.4d", i);
//		FILE *fptr = fopen(laser_list_file,"r");
//		double x,y, theta;
//		double range, angle;
//		fscanf(fptr, "%lf %lf %lf", &x, &y, &theta);
//
//		for(int j = 0; j < n_lasers_per_frame; j++){
//			fscanf(fptr, "%lf %lf", &range, &angle);
//			//if(j == 159){
//				Pose2 pose(x,y, theta+angle);
//
//				occupancyGrid.addLaser(pose, range);
//			//}
//		}
//		fclose(fptr);
//
//	}
//
//
////	LaserFactor::Occupancy occupancy = occupancyGrid.emptyOccupancy();
////	EXPECT_LONGS_EQUAL(900, occupancyGrid.laserFactorValue(0,occupancy));
////
////
////	occupancy[16] = 1;
////	EXPECT_LONGS_EQUAL(1, occupancyGrid.laserFactorValue(0,occupancy));
////
////	occupancy[15] = 1;
////	EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));
////
////	occupancy[16] = 0;
////	EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));
//
//
//	//run MCMC
//	OccupancyGrid::Marginals occupancyMarginals = occupancyGrid.runMetropolis(50000);
//	//EXPECT_LONGS_EQUAL( (width*height)/pow(resolution,2), occupancyMarginals.size());
//	//select a cell at a random to flip
//
//
//	printf("\n");
//	for(size_t i = 0, it = 0; i < occupancyGrid.height(); i++){
//		for(size_t j = 0; j < occupancyGrid.width(); j++, it++){
//			printf("%.2lf ", occupancyMarginals[it]);
//		}
//		printf("\n");
//	}
//
//	char marginalsOutput[1000];
//	sprintf(marginalsOutput, "/home/brian/Desktop/research/user/bpeasle/code/KinectInterface/marginals.txt");
//	FILE *fptr = fopen(marginalsOutput, "w");
//	fprintf(fptr, "%d %d\n", occupancyGrid.width(), occupancyGrid.height());
//
//	for(int i = 0; i < occupancyMarginals.size(); i++){
//		fprintf(fptr, "%lf ", occupancyMarginals[i]);
//	}
//	fclose(fptr);
//
//}

