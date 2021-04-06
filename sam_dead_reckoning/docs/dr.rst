=========================
SAM Dead Reckoning System
=========================

.. contents:: Table of Contents
   :depth: 2

Background
==========

The aim of this document is to explain the SAM Dead Reckoning (DR) system.
Our goal is to describe the following things in detail:

* A brief theoretical background
* How the robot_localization package is configured
* Processing of all sensor data before being fed into DR
* Process for calibrating sensor covariances
* Our motion model
* Process for calibrating the motion model
* The difference between the local filter and global filter
