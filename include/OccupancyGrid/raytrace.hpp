
#pragma once
#include <boost/iterator/iterator_facade.hpp>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <stdexcept>
#include <limits>

namespace occgrid {
template <typename T>
inline int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename real_t, typename int_t>
class ray_trace_iterator
  : public 
    boost::iterator_facade<
    ray_trace_iterator<real_t, int_t>
    , std::pair<int_t, int_t>
    , boost::forward_traversal_tag
    , std::pair<int_t, int_t>
    > 
{
    private:
      typedef typename boost::iterator_facade<
        ray_trace_iterator<real_t, int_t>
        , std::pair<int_t, int_t>
        , boost::forward_traversal_tag
        , std::pair<int_t, int_t>
        > super_t;
      // Input arguments
      real_t 
        px_,
        py_,
        dx_, 
        dy_,
        origin_x_,
        origin_y_,
        cell_size_x_, 
        cell_size_y_;

      // intermediate variables for faster computation
      int_t dirx_, diry_; /// (-1, 0, 1) integral steps (direction)
      real_t ex_, ey_; /// distance to the nearest grid line
      real_t Tx_, Ty_; /// Maximum time to collision (from one grid line to next)

      // State of iterator
      int_t i_, j_; /// Grid index
      real_t tx_, ty_; /// time to collision to next grid line
    public:
      ray_trace_iterator(
          real_t px,
          real_t py,
          real_t dx,
          real_t dy,
          real_t origin_x,
          real_t origin_y,
          real_t cell_size_x, 
          real_t cell_size_y
          ) : 
        px_(px),
        py_(py),
        dx_(dx),
        dy_(dy),
        origin_x_(origin_x),
        origin_y_(origin_y),
        cell_size_x_(cell_size_x),
        cell_size_y_(cell_size_y)
      {
        using std::floor;
        using std::fabs;
        // shift coordinates 
        px = px - origin_x;
        py = py - origin_y;

        // current grid cell
        i_ = static_cast<int_t>(floor(px / cell_size_x));
        j_ = static_cast<int_t>(floor(py / cell_size_y));

        dirx_ = signum(dx);
        diry_ = signum(dy);

        // whether the grid line we are going to hit is floor() or ceil()
        // depends on the direction ray is moving
        // using the fact that ceil() = floor() + 1
        int_t floor_or_ceilx = (dirx_ > 0) ? 1 : 0;
        int_t floor_or_ceily = (diry_ > 0) ? 1 : 0;
#ifdef DEBUG
        printf("cell: (%i, %i), dxdy:(%f, %f)\n", i, j, dx, dy);
        //std::cout << "cell size:" << cell_size_ << "pos:" << position << std::endl;
#endif
        // distance to nearest grid line
        ex_ = fabs((i_ + floor_or_ceilx) * cell_size_x - px);
        ey_ = fabs((j_ + floor_or_ceily) * cell_size_y - py);

        // (max) time to collision from one grid line to another
        Tx_ = (dx == 0) ? std::numeric_limits<real_t>::infinity() : cell_size_x / fabs(dx);
        Ty_ = (dy == 0) ? std::numeric_limits<real_t>::infinity() : cell_size_y / fabs(dy);

        // time to collision from this position
        tx_ = (dx == 0) ? std::numeric_limits<real_t>::infinity() : ex_ / fabs(dx);
        ty_ = (dy == 0) ? std::numeric_limits<real_t>::infinity() : ey_ / fabs(dy);

        if ( ! ((tx_ >= 0) && (ty_ >= 0))) {
          printf("t:(%f, %f), direction:(%f, %f), position:(%f, %f), cell:(%d, %d), cellsize:(%f, %f)\n", 
              tx_, ty_, dx, dy, px, py, i_, j_, cell_size_x, cell_size_y);
          throw std::logic_error("tx < 0 or ty < 0");
        }

        // time is always positive 
        assert(tx_ >= 0);
        assert(ty_ >= 0);
      }

      typename super_t::reference dereference() const {
        return std::make_pair(i_, j_);
      }

      bool equal(ray_trace_iterator it) const {
        return ((it.i_ == i_) && (it.j_ == j_) &&
          (it.tx_ == tx_) && (it.ty_ == ty_) &&
          (it.Tx_ == Tx_) && (it.Ty_ == Ty_) &&
          (it.dirx_ == dirx_) && (it.diry_ == diry_));
      }

      void increment() {
          if (tx_ < ty_) {
            i_ += dirx_;
            ty_ = ty_ - tx_;
            tx_ = Tx_;
          } else {
            j_ += diry_;
            tx_ = tx_ - ty_;
            ty_ = Ty_;
          }
      }

      std::pair<real_t, real_t>
        real_position() const {

          // whether the grid line we are going to hit is floor() or ceil()
          // depends on the direction ray is moving
          int_t floor_or_ceilx = (dirx_ > 0) ? 1 : 0;
          int_t floor_or_ceily = (diry_ > 0) ? 1 : 0;

          real_t ex = (dx_ == 0) ? ex_ // error is same as starting point
            : tx_ * fabs(dx_);
          real_t ey = (dy_ == 0) ? ey_ 
            : ty_ * fabs(dy_);

          real_t px = (i_ + floor_or_ceilx) * cell_size_x_ - ex * dirx_;
          real_t py = (j_ + floor_or_ceily) * cell_size_y_ - ey * diry_;

          // shift coordinates 
          px = px + origin_x_;
          py = py + origin_y_;

          return std::make_pair(px, py);
      }
};
} // namespace occgrid
