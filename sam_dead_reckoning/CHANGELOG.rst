^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sam_dead_reckoning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#2 <https://github.com/smarc-project/smarc_navigation/issues/2>`_ from smarc-project/noetic-devel
  Update to latest version of smarc_navigation with DR integrated to SIM
* Merge pull request `#61 <https://github.com/smarc-project/smarc_navigation/issues/61>`_ from smarc-project/dr_sim
  Updated sim to use DVL as twist and made installable
* Fixed install of params
* Updated sam_dead_reckoning to be installable
* Changed DVL to be fed in as twist instead of weird odom
* Merge pull request `#58 <https://github.com/smarc-project/smarc_navigation/issues/58>`_ from smarc-project/dr_sim
  Start of getting DR to work with new standard and DR
* Update dual_ekf_test.launch
* Update acc_model.py
* Merge pull request `#60 <https://github.com/smarc-project/smarc_navigation/issues/60>`_ from svbhat/noetic-devel
  Included tf_convenience_topics in DR launch file.
* corrected namespacing for DVL topic.Updated odom topic.
* Updated dependency to tf_convenience_topics
* added tf_convenenience_topics to dead reckoning launch file.
* Merge pull request `#1 <https://github.com/smarc-project/smarc_navigation/issues/1>`_ from smarc-project/noetic-devel
  Updates to check if smarc_navigation installation works for tf_convenience_topics
* Start of getting DR to work with new standard and DR
* Merge pull request `#56 <https://github.com/smarc-project/smarc_navigation/issues/56>`_ from ShounakCy/noetic-devel
  Comments updated
* Comments updated
* Merge pull request `#55 <https://github.com/smarc-project/smarc_navigation/issues/55>`_ from ShounakCy/noetic-devel
  changed cola2_msgs to smarc_msgs
* changed cola2_msgs to smarc_msgs
* Create dr.rst
* Merge pull request `#53 <https://github.com/smarc-project/smarc_navigation/issues/53>`_ from ShounakCy/noetic-devel
  Added comments in the dual_ekf_test.launch file
* added entries of the state
* added comments in the dual_ekf_test.launch file
* Fix init odom msg reception
* Merge pull request `#47 <https://github.com/smarc-project/smarc_navigation/issues/47>`_ from Jollerprutt/fix_nacho
  fix message type, topic and usage
* fix message type, topic and usage
* Merge pull request `#43 <https://github.com/smarc-project/smarc_navigation/issues/43>`_ from smarc-project/tf_lat_lon
  Added beginnings of lat lon conversion
* Added lat lon conversion to DR launch file
* Merge remote-tracking branch 'origin/noetic-devel' into tf_lat_lon
* Merge pull request `#46 <https://github.com/smarc-project/smarc_navigation/issues/46>`_ from smarc-project/fix_deps
  Fix deps from Nacho's changes
* Update package.xml
* Update CMakeLists.txt
* Merge pull request `#45 <https://github.com/smarc-project/smarc_navigation/issues/45>`_ from ignaciotb/noetic-devel
  Updated msg types
* Updated msg types
* Merge pull request `#44 <https://github.com/smarc-project/smarc_navigation/issues/44>`_ from ignaciotb/noetic-devel
  Noetic devel
* Using GPS odometry from navsat
* Changed msg type to dual RPM feedback
* Made stuff work on python3
* Update package.xml
* Update package.xml
* Merge pull request `#39 <https://github.com/smarc-project/smarc_navigation/issues/39>`_ from KKalem/asko_working_version
  last version from asko before we leave
* last version from asko before we leave
* Merge remote-tracking branch 'origin/master'
* Terribly simple, untested acc model for EKF
* Merge pull request `#38 <https://github.com/smarc-project/smarc_navigation/issues/38>`_ from KKalem/master
  changes from sam hw
* Changes from sam
* fixed yaw offset merge
* Fixed orientation offset from SBG
* Orientation 90deg fix
* Merge branch 'master' of https://github.com/smarc-project/smarc_navigation
* robot name changed from not sam to sam
* DVL DR will now be reset to current filtered DR when DVL starts/stops
* Fixed sbg orientation
* Fixed naming on DVL DR node
* Merge branch 'dual_ekf_test'
* Fixed robot_name param
* Merge pull request `#36 <https://github.com/smarc-project/smarc_navigation/issues/36>`_ from ignaciotb/dual_ekf_test
  Dual ekf test
* DR node based on DVL and SBG (forgot to actually add it)
* DR node based on DVL and SBG
* Added param to chose between bcing map-->odom from global EKF or static tf
* Added SBG and STIM imus to dual EKFs. Tested on sim
* Dummy motion model for tank test without GPS or DVL
* Fixed euler conversion
* 2d mode for global filter
* Increased covs
* Fusing DR from motion model
* New motion model running on SAM. Not added to EKF yet
* 3D motion model for DR with RPM + IMU
* Fliped rest of values
* Removed 2dmode used for testing
* Cleaned up for SAM. Tested
* SBG driver out of dr ns
* Fixed namespacing for navsat
* Removed IMU acc
* Addapted for real SAM
* corrected SBG orientation for ENU
* Fix NED frame on SBG imu
* Added call to custom forked SBG msg
* Added deps for SBG
* utm->map->odom->base tree running with GPS and compass
* Dual EKF with navsat setup. Missing SBG Imu data on ROS type
* Covs adjusted
* Tests with DVL asynch firing fused on EKF
* Launch file for DR without GPS
* Added DVL to twist converter to gps_dr launch
* Merge remote-tracking branch 'origin/gps_dr' into gps_dr
* IMU config back to normal on the filter
* Added some fixes
* Added the nodes needed for GPS DR
* increased the depth uncertainty
* Added better sam coefficient
* Added a new filter for running with continuous DR
* dvl2twist fixed indentation
* Motion model listeing to RPM combined feedback
* Motion model updated to be used as control on EKF
* Motion model input as control to EKF. This IMU config should work on the real SAM
* Updated EKFs parameters
* robot localization nodes set up. EKF map still drifting in simulation
* New EKF instance to provide world --> odom tf filtering the GPS
* Fixed sam/odom frame naming
* Moved DVL and IMU drivers to sam_core.launch
* Motion model for SAM to be integrated in the EKF
* Merged with dr_imu branch
* dr_node to integrate IMU accelerations out of the EKF
* Node to parse dvl to twist msgs now
* Node to parse dvl to twist msgs
* Merge pull request `#30 <https://github.com/smarc-project/smarc_navigation/issues/30>`_ from smarc-project/new_topics
  Changed to new SAM topics
* Merge pull request `#32 <https://github.com/smarc-project/smarc_navigation/issues/32>`_ from ignaciotb/new_topics
  New STIM drivers tested on separate IMU
* Pre-integration of acc from the IMU. Not running yet
* Added manual setting of position through rviz
* Fixed the gps dummy thing
* New STIM drivers tested on separate IMU
* Merge remote-tracking branch 'origin/new_topics' into new_topics
* Removed the GPS since it is now in core
* Changed to new topics
* Re-added the usage of pitch and roll angles from inclination
* Merge pull request `#23 <https://github.com/smarc-project/smarc_navigation/issues/23>`_ from ignaciotb/master
  New covs, add limits for press values and removed warning for pub que…
* Merge branch 'master' into master
* Merge pull request `#29 <https://github.com/smarc-project/smarc_navigation/issues/29>`_ from Jollerprutt/master
  Fix wrong gravitational_acceleration for stim_dr
* Merged with smarc_navigation. SAM orientation from IMU gyros only
* IMU orientation from integrating gyros only
* Nälsta Pool Test latest version Nov 2019
* Merged fixing_imu branch
* Merge remote-tracking branch 'ignaciotb/master'
* Changed the IMU input flags to adjust to change in IMU orientation
* Merge branch 'master' of https://github.com/smarc-project/smarc_navigation
* Fix wrong gravitational_acceleration
* Merge remote-tracking branch 'origin/master' into dead_reckoning
* DR tested offline with rosbags. Ready to test on SAM
* Merge pull request `#28 <https://github.com/smarc-project/smarc_navigation/issues/28>`_ from nilsbore/dummy_gps
  Add dummy world_utm -> sam_odom tf transformation
* Forgot to uncomment nmea navsat driver
* Added dummy world_utm -> sam_odom tf transformation
* Fixed the spoofing nodes a bit
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#27 <https://github.com/smarc-project/smarc_navigation/issues/27>`_ from nilsbore/fix_orientation
  Fixed DR for new sam imu orientation
* Fixed DR for new sam imu orientation
* Added two nodes to be able to run dead reckoning without some ensors
* Transfrom from press sensor to base_link now done through tf
* Adjusting IMU inputs on filter
* Added imu_link to tf tree
* SAM depth transform tested
* Depth transform to base link tested
* Changed control loop freq
* Depth transform from depth frame to base frame
* Latest changes from laptop
* Correction for depth. Not working yet
* Remapped depth sensor and changed EKF freq
* Merged with remote
* Added transformation from depth sensor to base link
* New covs, add limits for press values and removed warning for pub queue size
* Merge pull request `#22 <https://github.com/smarc-project/smarc_navigation/issues/22>`_ from ignaciotb/master
  Pressure to depth converter
* press_to_depth added to stim DR launch
* Added rosparams
* Pressure to depth conv node
* Fixed something
* Fixed missing $ in launch DR
* Merge pull request `#21 <https://github.com/smarc-project/smarc_navigation/issues/21>`_ from ignaciotb/master
  Moved tf pub to sam_core. Added rosparams for topics and frames
* Moved tf pub to sam_core. Added rosparams for topics and frames
* Renamed a bunch of stuff
* Contributors: Carl Ljung, Jollerprutt, Nacho, Nils Bore, Ozer, Shounak, Torroba, ignaciotb, svbhat, torroba, xyp8023
