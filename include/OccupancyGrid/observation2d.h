
#pragma once
template <typename real_t>
class _Observation2D {
  public:
    real_t px, py, ptheta, range;

    _Observation2D() {}

    _Observation2D(real_t px, 
        real_t py,
        real_t ptheta,
        real_t range) : px(px),py(py), ptheta(ptheta), range(range) {}
};
typedef _Observation2D<double> Observation2D;
