#include "OccupancyGrid/utility.hpp"
#include <gtest/gtest.h>

using namespace occgrid;
TEST(logdouble, addition) {
  logdouble ld1 = logdouble::from_energy(6.238);
  logdouble ld2 = logdouble::from_energy(6.238);
  logdouble zero(0);
  logdouble one(1);
  logdouble remaining = one - ld1 - ld2 - zero;
  //std::cout << "remaining:" << remaining << std::endl;
  logdouble backtoone = remaining + ld1 + ld2 + zero;
  //std::cout << "backtoone:" << backtoone << std::endl;
  ASSERT_DOUBLE_EQ(todouble(one), todouble(backtoone)) << "backtoone:" << backtoone;
}
TEST(logdouble, multiplication) {
  logdouble ld1 = logdouble::from_energy(6.238);
  logdouble ld2 = logdouble::from_energy(6.238);
  logdouble one(1);
  logdouble remaining = one * ld1 * ld2;
  //std::cout << "remaining:" << remaining << std::endl;
  logdouble backtoone = remaining / ld1 / ld2;
  //std::cout << "backtoone:" << backtoone << std::endl;
  ASSERT_DOUBLE_EQ(todouble(one), todouble(backtoone)) << "backtoone:" << backtoone;
}
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
