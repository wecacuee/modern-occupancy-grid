#ifndef __LASER_FACTOR__
#define __LASER_FACTOR__

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>

using namespace std;
using namespace gtsam;

/**
 * Laser Factor
 * @brief factor that encodes a laser measurements likelihood.
 */

class LaserFactor : public DiscreteFactor{
private:
	vector<Index> 	cells_;	///cells in which laser passes through

public:

	///constructor
	LaserFactor(const vector<Index> &cells) : cells_(cells) {}

	/// Find value for given assignment of values to variables
	virtual double operator()(const Values &vals) const;

	/// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
	virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const{
		throw runtime_error("operator * not implemented");
	}

	virtual operator DecisionTreeFactor() const{
		throw runtime_error("operator DecisionTreeFactor not implemented");
	}

  virtual DecisionTreeFactor toDecisionTreeFactor() const {
		throw runtime_error("toDecisionTreeFactor not implemented");
  }
};


#endif
