#pragma once

#include <boost/iterator/transform_iterator.hpp>
#include <boost/operators.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/utility/result_of.hpp>
#include <boost/assert.hpp>

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

class logdouble : 
  boost::multipliable<logdouble>,
  boost::addable<logdouble>,
  boost::subtractable<logdouble>,
  boost::dividable<logdouble>,
  boost::less_than_comparable<logdouble>,
  boost::equality_comparable<logdouble>
  {
    private:
    double neglog_;
    static double fromdouble(double d) {
      return -std::log(d);
    }
  public:
  bool operator<(const logdouble& ld) const {
    return ! (neglog_ < ld.neglog_);
  }
  bool operator==(const logdouble& ld) {
    return (neglog_ == ld.neglog_);
  }

  logdouble operator*=(const logdouble& ld) {
    neglog_ += ld.neglog_;
    return *this;
  }

  logdouble operator+=(const logdouble& ld) {
    double minlog = (neglog_ < ld.neglog_) ? neglog_ : ld.neglog_;
    if (abs(neglog_ - ld.neglog_) < 10) {
      double d1 = std::exp(-(neglog_ - minlog));
      double d2 = std::exp(-(ld.neglog_ - minlog));
      neglog_ = fromdouble(d1 + d2) + minlog;
    } else
      neglog_ = minlog;
    return *this;
  }

  logdouble operator-=(const logdouble& ld) {
    BOOST_ASSERT(neglog_ <= ld.neglog_);
    double minlog = neglog_;
    if (abs(neglog_ - ld.neglog_) < 10) {
      double d1 = std::exp(-(neglog_ - minlog));
      double d2 = std::exp(-(ld.neglog_ - minlog));
      neglog_ = fromdouble(d1 - d2) + minlog;
    } else
      neglog_ = minlog;
    return *this;
  }

  logdouble operator/=(const logdouble& ld) {
    neglog_ -= ld.neglog_;
    return *this;
  }
  friend std::ostream& operator << (std::ostream& os, const logdouble& r);
  explicit logdouble() : neglog_(0) {};
  explicit logdouble(double d) : neglog_(fromdouble(d)) {
    BOOST_ASSERT(d >= 0); 
  };
  static logdouble from_energy(double w) {
    logdouble ld;
    ld.neglog_ = w;
    return ld;
  }
  double todouble() const {
    return std::exp( - neglog_);
  }
  bool isnan() const {
    return std::isnan(neglog_);
  }
};

bool isnan(const logdouble& ld);

template<typename T>
T toprobability(double w) {
  return std::exp(-w);
}
template <>
logdouble toprobability<logdouble>(double w);
template <typename T>
double todouble(T t) {
  return static_cast<double>(t);
}
template <>
double todouble<logdouble>(logdouble t);


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

bool isnan(const SymReal& sr);
} // namespace occgrid
