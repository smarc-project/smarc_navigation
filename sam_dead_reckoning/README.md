# README

Short README on what to do to run the dead reckoning when using a ROS bag instead of directly on SAM.

## Before Launch
- Uncomment the robot description part in `sam_dr.launch`
- Filter the rosbag and remove all `/tf` and `/tf_static`. The DR node will provide them

## After Launch
In order to kick things off, you need to publish a few topics that aren't necessarily in the rosbags, esp.
when collecting them in the lab rather than underwater.

1. DVL: 
    rostopic pub /sam/core/dvl smarc_msgs/DVL "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    velocity: {x: 0.0, y: 0.0, z: 0.0}
    velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    altitude: 0.0
    beams:
    - range: 0.0
    range_covariance: 0.0
    velocity: 0.0
    velocity_covariance: 0.0
    pose:
        header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
        pose:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}"
2. GPS Fix:
    rostopic pub /sam/core/gps sensor_msgs/NavSatFix "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    status: {status: 0, service: 0}
    latitude: 0.0
    longitude: 0.0
    altitude: 0.0
    position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    position_covariance_type: 0"
-> Could be that you need to publish this twice
