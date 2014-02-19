============
Introduction
============

This repository is the code for the paper titled:

  Modern MAP inference methods for accurate and faster occupancy grid mapping on higher order factor graphs 
  by V. Dhiman and A. Kundu and F. Dellaert and J. J. Corso 

Please refer to the
[paper](http://www.acsu.buffalo.edu/~vikasdhi/images/modern_map.pdf) for more
details.


======================
Repeatable experiments
======================

Dependencies
------------
You will need

1. Boost 1.48
	sudo apt-add-repository ppa:jkeiren/ppa
	sudo apt-get update
	sudo apt-get install libboost*1.48-dev

2. OpenCV 2.4 
	Install from source http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html

3. GTSAM 2.3
	Install from source https://borg.cc.gatech.edu/download

4. Gtest

5. You can optionally install player and stage to generate simulated data
again, which is enclosed.


Building
~~~~~~~~
1. All the development and testing has been done on Ubuntu 12.04. 

2. Use standard cmake procedure for building binaries
	mkdir build
	cd build
	cmake ..
	make

3. All executable binaries are created in "bin" folder in the root directory.

4. The data required for the paper is available by running the python script
   wgetdata.py in Data/ directory.

    cd Data/; python wgetdata.py


Convergence Experiments
~~~~~~~~~~~~~~~~~~~~~~~

You can run the convergence experiment on "cave" dataset. The following
scripts will run five algorithms on cave dataset:

	python scripts/run_experiments.py scripts/conv_exp_cave.py

Similarly, for other datasets use the following scripts:

	python scripts/run_experiments.py scripts/conv_exp_hospital_section.py
	python scripts/run_experiments.py scripts/conv_exp_albertb.sm.py


These scripts will save qualitative results in the respective data
directories as *.png files.
These scripts will save *-plot-time-energy.npy files in the respective data
directories. This data can be plotted by using the following scripts:

	python scripts/plot_time_energy.py scripts/conv_exp_cave.py
	python scripts/plot_time_energy.py scripts/conv_exp_hospital_section.py
	python scripts/plot_time_energy.py scripts/conv_exp_albertb.sm.py

Step size comparison for dual decomposition
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following scripts will run and plot the dual decompositon

	python scripts/run_experiments.py scripts/dual_decomposition_stepsize.py
	python scripts/plot_time_energy.py scripts/dual_decomposition_stepsize.py
