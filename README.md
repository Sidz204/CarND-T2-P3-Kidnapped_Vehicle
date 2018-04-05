# Kidnapped Vehicle Project

In this project the aim was to implement a 2 dimensional particle filter in C++. Particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step filter will also get observation and control data.

### Files & Directories:
- src : it contains all the main C++ files which involes the implementation of Particle filter.
- build : it contains all the files necessary to compile and run the code.
- data : it contains the map data which includes the position of landmarks.

particle_filter.cpp : its the file to look out for where the main implementation of particle filter is done.

### Instructions for running the code:
1. cd build
2. cmake ..
3. make
4. ./particle_filter

or 
simple run the ./run.sh script. 

and then launch the simulator


Download Links for Term 2 Simulator: [link](https://github.com/udacity/self-driving-car-sim/releases)
