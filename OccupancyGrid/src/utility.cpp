#include "utility.hpp"

namespace occgrid {
std::ostream& operator << (std::ostream& os, const logdouble& r) {
   os << "exp(" << -r.neglog_ << ")";
   return os;
}
// template struct addable<logdouble>;
// template struct subtractable<logdouble>;
// template struct multipliable<logdouble>;
// template struct dividable<logdouble>;
//
template<>
logdouble toprobability<logdouble>(double w) {
  return logdouble::from_energy(w);
}
template <>
double todouble<logdouble>(logdouble t) {
  return t.todouble();
}

bool isnan(const logdouble& ld) {
  return ld.isnan();
}

SymReal& operator*=(SymReal& r1, SymReal& r2) {
  if (r2.expression_ == "1") 
    r1.expression_ = r1.expression_ ;
  else if (r1.expression_ == "1") 
    r1.expression_ = r2.expression_ ;
  else {
    // std::string eb = (r1.depth_ > 0) ? "]" : "";
    // std::string sb = (r1.depth_ > 0) ? "[" : "";
    // r1.expression_ = sb + r1.expression_ + eb ;
    // eb = (r2.depth_ > 0) ? "]" : "";
    // sb = (r2.depth_ > 0) ? "[" : "";
    // r2.expression_ = sb + r2.expression_ + eb;
    r1.expression_ += r2.expression_;
  }
  return r1;
}

SymReal& operator+=(SymReal& r1, SymReal& r2) {
  if (r2.expression_ == "0") 
    r1.expression_ = r1.expression_ ;
  else if (r1.expression_ == "0") 
    r1.expression_ = r2.expression_ ;
  else {
    if ((r1.expression_[0] == '[') && (r1.expression_[r1.expression_.size() - 1] == ']'))
      r1.expression_[0] = ' ', r1.expression_[r1.expression_.size() - 1] = ' ';

    r1.expression_ = "[" + r1.expression_ + " + " + r2.expression_ + "]";
  }
  return r1;
}

std::ostream& operator << (std::ostream& os, const SymReal& r) {
  os << r.expression_;
  return os;
}

SymReal& operator/=(SymReal& r1, SymReal& r2) {
  return r1; // cheating. We don't want normalization
}

bool isnan(const SymReal& ld) {
  return false;
}
}
