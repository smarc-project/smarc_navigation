# smarc_navigation
![CI](https://github.com/smarc-project/smarc_navigation/workflows/CI/badge.svg?branch=noetic-devel) [![license](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

Visual odometry, localization and mapping components for the SMARC systems

## auv_ekf_localization
This package implements a localization solution for an AUV based on an EKF filter with inputs from an IMU, DVL and a MBES with a given feature-based map.

The output of the main node is an estimate of the pose of the AUV, defined by its 6DOF, together with the updated tf tree.
This package also contains a service provider node that hands out the map when requested and a handler for MBES inputs which extracts landmarks from incoming MBES sensory input.

In order to run it, set the flag "navigation_on" to true in the launch file in the smarc_bringup package.
