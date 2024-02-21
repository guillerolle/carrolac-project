#!/usr/bin/bassystem support or before trying with your own collected data. The machine hall datasets have the MAV being picked up in the beginning and then set down, we normally skip this part, but it should be able to be handled by the filter if SLAM features are enabled. Please take a look at the run_ros_eth.sh script for some reasonable default values (they might still need to be tuned).h
for x in 3.25 3.15 3.35
do
  for y in 2.25 2.15 2.35
  do
     for z in 0.25 0.20 0.30
     do
        for skew in 0.0 -0.2 0.2
        do
          ign service --service /world/empty/control\
            --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 500 --req\
             'pause: true'

          ign service --service /world/empty/set_pose\
            --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 500 --req\
            "name: 'CHECKERBOARD', position: {x: $x, y: $y, z: $z}, orientation: {x: $skew, y: $skew, z: 3.14}"

          ign service --service /world/empty/control\
            --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 500 --req\
             'pause: false'
          sleep .2
        done
     done
  done
done