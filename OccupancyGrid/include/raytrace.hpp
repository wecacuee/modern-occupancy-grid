
#pragma once
#include <boost/iterator/iterator_facade.hpp>
#include "observation2d.h"
#include "occgrid.h"

template <typename Map>
class ray_trace_iterator
  : public 
    boost::iterator_facade<
    ray_trace_iterator<Map>
    , std::pair<typename Map::int_t, typename Map::int_t>
    , boost::forward_traversal_tag
    > 
{
    private:
      const Map& map_;
      typedef typename Map::real_type real_t;
      typedef typename Map::integer_type int_t;
      const _Observation2D<real_t>& pose_;
      real_t max_range_;
      int_t i_, j_;
      real_t tx_, ty_;
      int_t dirx_, diry_;
      real_t Tx_, Ty_;
      int_t maxsizex_, maxsizey_;
      int_t n_;
    public:
      ray_trace_iterator() {};

      ray_trace_iterator(const Map& map,
          const Observation2D& pose,
          real_t max_range) : map_(map), pose_(pose), max_range_(max_range) {
        real_t px(pose.px);
        real_t py(pose.py);
        real_t ptheta(pose.ptheta);
        // shift coordinates so that we are always in first quadrant
        px = px - map_.min_pt_(0);
        py = py - map_.min_pt_(1);

        real_t dx = cos(ptheta);
        real_t dy = sin(ptheta);
        int_t dirx = signum(dx);
        dirx = (dirx == 0) ? 1 : dirx;

        int_t diry = signum(dy);
        diry = (diry == 0) ? 1 : diry;
        int_t i = static_cast<int_t>(floor(px / map_.cell_size_(0)));

        int_t j = static_cast<int_t>(floor(py / map_.cell_size_(1)));

        // distance to nearest grid line
        int_t floor_or_ceilx = (dirx > 0) ? 1 : 0;
        int_t floor_or_ceily = (diry > 0) ? 1 : 0;
#ifdef DEBUG
        printf("cell: (%i, %i), dxdy:(%f, %f)\n", i, j, dx, dy);
        //std::cout << "cell size:" << cell_size_ << "pos:" << position << std::endl;
#endif
        real_t ex = dirx * ((i + floor_or_ceilx) * map_.cell_size_(0) - px);
        real_t ey = diry * ((j + floor_or_ceily) * map_.cell_size_(1) - py);

        // time to collision from one grid line to another
        real_t Tx = map_.cell_size_(0) / fabs(dx);
        real_t Ty = map_.cell_size_(1) / fabs(dy);

        // time to collision
        real_t tx = ex / fabs(dx);
        real_t ty = ey / fabs(dy);
        // assert(tx >= 0); // time is always positive (for me, for now)
        if ( ! ((tx >= 0) && (ty >= 0))) {

          printf("direction:(%f, %f), position:(%f, %f), cell:(%d, %d), cellsize:(%f, %f)\n", 
              dx, dy, px, py, i, j, map_.cell_size_(0), map_.cell_size_(1));
          throw std::logic_error("tx < 0 or ty < 0");
        }
        assert(tx >= 0);
        assert(ty >= 0);

        real_t dirmag = sqrt(dx*dx + dy*dy); 
        real_t n = floor(max_range * fabs(dx) / dirmag / map_.cell_size_(0)) 
          + floor(max_range * fabs(dy) / dirmag / map_.cell_size_(1));
        maxsizex_ = map_.og_.size[0];
        maxsizey_ = map_.og_.size[1];
        i_ = i;
        j_ = j;
        tx_ = tx;
        ty_ = ty;
        Tx_ = Tx;
        Ty_ = Ty;
        dirx_ = dirx;
        diry_ = diry_;
        n_ = n;
      }

      static ray_trace_iterator end_iterator() {
        return ray_trace_iterator();
      }

      std::pair<int_t, int_t> dereference() {
        return std::make_pair(i_, j_);
      }
      bool equal(ray_trace_iterator it) {
        return (it.i_ == i_) && (it.j_ == j_) &&
          (it.map_ == map_) && (it.pose_ == pose_) && (it.max_range_ == max_range_);
      }

      void increment() {
        if (n_ > 0) {
          n_--;

          if (tx_ < ty_) {
            i_ += dirx_;
            ty_ = ty_ - tx_;
            tx_ = Tx_;
          } else {
            j_ += diry_;
            tx_ = tx_ - ty_;
            ty_ = Ty_;
          }

          if (i_ < 0 ||  j_ < 0 || i_ >= maxsizex_ || j_ >= maxsizey_ ||
              map_.is_occupied(i_, j_))
          {
            *this = end_iterator();
          }
        } 
        *this = end_iterator();
      }
};
