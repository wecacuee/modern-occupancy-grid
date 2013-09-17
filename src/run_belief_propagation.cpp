#include "OccupancyGrid/gtsam_adaptor.hpp"
#include "OccupancyGrid/sumproduct.hpp"
#include "OccupancyGrid/cvmat_serialization.h"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/BPLaserFactor.hpp"
#include "OccupancyGrid/TwoAssumptionAlgorithm.h"
#include "OccupancyGrid/loadOccupancyGrid.h"
#include <boost/foreach.hpp>
#include <boost/typeof/typeof.hpp>

#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;


using namespace std;
using namespace gtsam;
using namespace occgrid;

typedef G<OccupancyGrid, logdouble, size_t, gtsam::Index> OccupancyGridGraph;
namespace occgrid {
  namespace detail {
      // full template specialization for overriding the sumproduct update
      // mechanism
      template <> logdouble
      belief
      < OccupancyGridGraph,
        typename OccupancyGridGraph::MessageValues,
        typename OccupancyGridGraph::SampleSpaceMap,
        typename OccupancyGridGraph::FactorMap>
        (
         typename boost::graph_traits<OccupancyGridGraph>::edge_descriptor e,
         const OccupancyGridGraph  &fg,
         typename OccupancyGridGraph::MessageValues &msgs,
         typename OccupancyGridGraph::SampleSpaceMap &cdmap,
         typename OccupancyGridGraph::FactorMap& fmap, 
         //const typename Assignment::value_type &xv
         const typename OccupancyGridGraph::SampleSpaceMap::value_type::first_type::value_type &xv,
         f2v_edge_tag
        ) {
          BOOST_AUTO_TPL(factor, source(e, fg));
          BOOST_AUTO_TPL(cell, target(e, fg));
          return fg.compute_belief(factor, cell, xv, msgs);
        }
  }
}

Visualiser global_vis_;

template <typename FactorGraph, typename MessageValues>
struct display_peridically_visitor 
: public boost::base_visitor<display_peridically_visitor<FactorGraph, MessageValues> >
{
  typedef boost::on_examine_edge event_filter;
  display_peridically_visitor(const MessageValues& msgs) : msgs_(msgs), count_(0), clock_start_(clock()) {}
  void operator()(typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph& fg) {
    if ((count_ % 500000 == 0)) {
      std::cout << "Iterations:" << count_ << std::endl;
      display(fg);
    }
    ++count_;
  }
  void display(const FactorGraph& fg) {
    /// Very very slow
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    typedef typename boost::property_traits<MessageValues>::value_type value_type;
    std::vector<value_type> margs(num_variables(fg));
    gtsam::Assignment<gtsam::Index> best_assign;
    size_t count = 0;
    BOOST_FOREACH(vertex_descriptor u, variables(fg)) {
      value_type prod_0(marginal(fg, msgs_, u, 0));
      value_type prod_1(marginal(fg, msgs_, u, 1));
      value_type denom = prod_0 + prod_1;
      if (denom == value_type(0)) 
        margs[count] = value_type(0);
      else
        margs[count] = prod_1 / denom;
      best_assign[count] = (margs[count] < value_type(0.5)) ? 0 : 1;
      ++count;
    }
    fg.display(global_vis_, margs);
    double energy = fg.g_(best_assign);
    clock_t et = clock();
    std::cout << "<Energy>\t" << ((float)(et - clock_start_)) / CLOCKS_PER_SEC << "\t" << energy << std::endl;
  }
  private:
  const MessageValues& msgs_;
  int count_;
  clock_t clock_start_;
};

int main(int argc, const char *argv[])
{
  global_vis_.enable_show();
  cv::namedWindow("c", cv::WINDOW_NORMAL);
  // parse arguments ///////////////////////////////////////////
  // Declare the supported options.
  po::options_description desc("Run dual decomposition");
  desc.add_options()
    ("help", "produce help message")
    ("resolution", po::value<double>()->required(), "Size of square cell in the map")
    ("width", po::value<double>(), "Width")
    ("height", po::value<double>(), "Height")
    ("dir", po::value<std::string>()->default_value("Data/player_sim_with_reflectance"), "Data directory")
    ("clock", po::value<double>()->default_value(400), "Max clock")
;

  po::positional_options_description pos;
  pos.add("resolution", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  std::string directory = vm["dir"].as<std::string>();
  double max_clock = CLOCKS_PER_SEC * vm["clock"].as<double>();
  // end of parse arguments ////////////////////////////////////

  // Create the occupancy grid data structure
  OccupancyGrid occupancyGrid = loadOccupancyGrid(vm);
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  global_vis_.enable_show();

  typename OccupancyGridGraph::MessageTypes::base_type msg_values(logdouble(0.5));
  typename OccupancyGridGraph::MessageValues msgs(msg_values);
  OccupancyGridGraph ogg(occupancyGrid);
  using occgrid::get;
  OccupancyGridGraph::SampleSpaceMap cdmap = get(sample_space_t(), ogg);
  OccupancyGridGraph::FactorMap fmap = get(factor_map_t(), ogg);
  OccupancyGridGraph::IsFactorMap is_factor = get(is_factor_t(), ogg);

  display_peridically_visitor<OccupancyGridGraph,
    typename OccupancyGridGraph::MessageValues > display_vis(msgs);

  // initialize belief propagation by two assumption algorithm
  typedef typename boost::graph_traits<OccupancyGridGraph>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_traits<OccupancyGridGraph::MessageValues>::value_type msg_value_type;
  LaserFactor::Occupancy occupancy = occupancyGrid.emptyOccupancy();
  std::vector<double> energy(occupancyGrid.cellCount());
  two_assumption_algorithm(occupancyGrid, occupancy, energy);
  using boost::adjacency_iterator;
  BOOST_FOREACH(vertex_descriptor x, variables(ogg)) {
    BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, ogg)) {
      gtsam::Index value =  occupancy[x];
      msgs[typename OccupancyGridGraph::MessageValues::key_type(x, f, value)] = msg_value_type(0.9);
      msgs[typename OccupancyGridGraph::MessageValues::key_type(x, f, 1 - value)] = msg_value_type(0.1);
    }
  }

  sumproduct_visitor<
    OccupancyGridGraph,
    OccupancyGridGraph::MessageValues,
    OccupancyGridGraph::SampleSpaceMap,
    OccupancyGridGraph::FactorMap,
    OccupancyGridGraph::IsFactorMap>
      spvis (msgs, cdmap, fmap, is_factor);


  BOOST_AUTO(vistor_list, std::make_pair(spvis, display_vis));
  // getting num_edges(ogg)
  BOOST_AUTO(n_iter, num_edges(ogg) * 1.0);
  std::cout << "Number of iterations:" << n_iter << std::endl;
  random_edge_traversal(ogg, vistor_list, n_iter, max_clock);
  display_vis.display(ogg);
  std::stringstream ss;
  ss << directory << "/run_belief_propagation.png";
  global_vis_.save(ss.str());
}
