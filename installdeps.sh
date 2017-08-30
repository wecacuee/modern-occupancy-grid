sudo apt-get install libhdf5-dev
make -f gtsam.mk
export GTSAM_DIR=$(pwd)/external/gtsam/lib/cmake/GTSAM/
make -f opengm.mk
make -f gtest.mk
export GTEST_ROOT=$(pwd)/external/gtest/
