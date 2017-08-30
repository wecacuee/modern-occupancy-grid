sudo apt-get install libboost-timer1.49-dev libboost1.49-dev libboost-serialization1.49-dev  libboost-system1.49-dev libboost-filesystem1.49-dev libboost-thread1.49-dev libboost-regex1.49-dev libboost-chrono1.49-dev libboost-program-options1.49-dev
make -f gtsam.mk
ln -fsT gtsam-2.4.0-install external/gtsam
make -f opengm.mk
ln -fsT opengm-2.3.6-install external/opengm
