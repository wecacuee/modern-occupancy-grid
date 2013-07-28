#pragma once

#include <boost/iterator/transform_iterator.hpp>
#include <boost/operators.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/utility/result_of.hpp>

#include <cmath>

namespace occgrid {
  // workaround to fix a bug with transform_iterator
  // http://stackoverflow.com/questions/3090270/how-to-use-a-phoenix-expression-with-boosttransform-iterator
  template <class UnaryFunc, class Iterator>
  boost::transform_iterator<
     UnaryFunc,
     Iterator,
     typename boost::result_of<
        UnaryFunc(typename std::iterator_traits<Iterator>::value_type)
     >::type
  >
  make_trans_it(Iterator it, UnaryFunc fun){
     return
        boost::transform_iterator<
           UnaryFunc,
           Iterator,
           typename boost::result_of<
              UnaryFunc(typename std::iterator_traits<Iterator>::value_type)
           >::type
        >(it, fun);
  };

class logdouble {
  double neglog_;
  public:
  logdouble operator*=(const logdouble& ld) {
    return neglog_ += ld.neglog_;
  }
  logdouble operator+=(const logdouble& ld) {
    using std::min;
    return min(neglog_, ld.neglog_);
  }
  logdouble operator-=(const logdouble& ld) {
    using std::min;
    return min(neglog_, ld.neglog_);
  }
  logdouble operator/=(const logdouble& ld) {
    return neglog_ -= ld.neglog_;
  }
  logdouble operator=(const double d) {
    return neglog_ = - std::log(d);
  }
  friend std::ostream& operator << (std::ostream& os, const logdouble& r);
  logdouble() : neglog_(0) {};
  logdouble(double d) : neglog_(- std::log(d)) {};
  static logdouble from_energy(double w) {
    logdouble ld;
    ld.neglog_ = w;
    return ld;
  }
};

template<typename T>
T toprobability(double w) {
  return std::exp(-w);
}

class SymReal {
private:
  std::string expression_;
  friend SymReal& operator*=(SymReal&, SymReal&);
  friend SymReal& operator+=(SymReal&, SymReal&);
  friend std::ostream& operator << (std::ostream& os, const SymReal& r);
  friend SymReal& operator/=(SymReal&, SymReal&);
public:
  SymReal() : expression_() {}
  SymReal(double r) : expression_( boost::lexical_cast<std::string>(r) ) {}
  SymReal(std::string exp) : expression_(exp) {}
};

} // namespace occgrid
