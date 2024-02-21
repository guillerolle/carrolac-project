#!/usr/bin/env python3

import subprocess
import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_2_homog(pose):
    rotmat = R.from_euler(seq='XYZ', angles=pose[3:6]).as_matrix()
    hommat = np.eye(4)
    hommat[0:3, 0:3] = rotmat
    hommat[0:3, 3] = pose[0:3]
    return hommat


def homog_2_pose(htf):
    roteul = R.from_matrix(htf[0:3, 0:3]).as_euler(seq="XYZ")
    pose = np.zeros(6)
    pose[0:3] = htf[0:3, 3]
    pose[3:6] = roteul
    return pose


def play_transformation(htf0, dtx=np.array([0]), dty=np.array([0]), dtz=np.array([0]), \
                        drx=np.array([0]), dry=np.array([0]), drz=np.array([0])):
    total_steps = len(dtx) * len(dty) * len(dtz) * len(drx) * len(dry) * len(drz)
    current_step = 1
    for _dtx in dtx:
        for _dty in dty:
            for _dtz in dtz:
                for _drx in drx:
                    for _dry in dry:
                        for _drz in drz:
                            print("\nComputing Transformation Step " + str(current_step) + "/" + str(total_steps) \
                                  + "(" + str(current_step / total_steps * 100) + "%)")
                            _tf02tf1 = htf0 @ pose_2_homog(np.array([_dtx, _dty, _dtz, _drx, _dry, _drz]))
                            # print(_tf02tf1)
                            # print(homog_2_pose(_tf02tf1))
                            # print(homog_2_pose(_tf02tf1 @ np.linalg.inv(_checker2transform)))
                            cbpose = homog_2_pose(_tf02tf1 @ np.linalg.inv(_checker2transform))

                            subprocess.call(["ign", "service",
                                             "--service", "/world/empty/control",
                                             "--reqtype", "ignition.msgs.WorldControl",
                                             "--reptype", "ignition.msgs.Boolean",
                                             "--timeout", "500",
                                             "--req", 'pause: true'])

                            subprocess.call(["ign", "service",
                                             "--service", "/world/empty/set_pose",
                                             "--reqtype", "ignition.msgs.Pose",
                                             "--reptype", "ignition.msgs.Boolean",
                                             "--timeout", "500",
                                             "--req",
                                             "name: 'CHECKERBOARD', " +
                                             "position: {x: " + str(cbpose[0]) +
                                             ", y: " + str(cbpose[1]) +
                                             ", z: " + str(cbpose[2]) +
                                             "}, " +
                                             "orientation: {x: " + str(cbpose[3]) +
                                             ", y: " + str(cbpose[4]) +
                                             ", z: " + str(cbpose[5]) +
                                             "}"
                                             ])

                            subprocess.call(["ign", "service",
                                             "--service", "/world/empty/control",
                                             "--reqtype", "ignition.msgs.WorldControl",
                                             "--reptype", "ignition.msgs.Boolean",
                                             "--timeout", "500",
                                             "--req", 'pause: false'])

                            subprocess.call(["sleep", ".01"])

                            current_step += 1


# POSE DEL CHECKERBOARD EN EL MUNDO DE GAZEBO
checkerboard_init_pose = np.array([3.25, 2.25, 0.25, 0, 0, 3.14])
_map2checker = pose_2_homog(checkerboard_init_pose)
print(_map2checker)

# FRAME RESPECTO AL CUAL SE TRANSFORMARA LA POSE DEL CHECKERBOARD
_checker2transform = pose_2_homog(np.array([1, 0, 0, 0, 0, -3.14]))
print(_checker2transform)

_map2transform = _map2checker @ _checker2transform
print(_map2transform)
print(homog_2_pose(_map2transform))

play_transformation(_map2transform, dtx=np.linspace(start=0.0, num=10, stop=-0.3))
play_transformation(_map2transform, dty=np.linspace(start=-0.5, num=10, stop=0.5))
play_transformation(_map2transform, dtz=np.linspace(start=-0.1, num=10, stop=0.2))
play_transformation(_map2transform, drx=np.linspace(start=-0.8, num=10, stop=0.8), dtx=np.array([-0.3]))
play_transformation(_map2transform, drx=np.array([0.8]), dtx=np.array([-0.3]),
                    dtz=np.linspace(start=-0.1, num=10, stop=0.2))

subprocess.call(["ign", "service",
                 "--service", "/world/empty/set_pose",
                 "--reqtype", "ignition.msgs.Pose",
                 "--reptype", "ignition.msgs.Boolean",
                 "--timeout", "500",
                 "--req",
                 "name: 'CHECKERBOARD', " +
                 "position: {x: " + str(checkerboard_init_pose[0]) +
                 ", y: " + str(checkerboard_init_pose[1]) +
                 ", z: " + str(checkerboard_init_pose[2]) +
                 "}, " +
                 "orientation: {x: " + str(checkerboard_init_pose[3]) +
                 ", y: " + str(checkerboard_init_pose[4]) +
                 ", z: " + str(checkerboard_init_pose[5]) +
                 "}"
                 ])
