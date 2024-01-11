from math import pi

flatPose = [0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0]

curlPose = [pi/2, pi/2, 2.0,
            -pi/2, -pi/2, 2.0,
            -pi/2, -pi/2, 2.0,
            pi/2, pi/2, pi/2]

crabStandPose = [-pi/2, pi/8, pi/6,
                 pi/2, -pi/8, pi/6,
                 pi/2, -pi/8, pi/6,
                 -pi/2, pi/8, pi/6]

dogReadyPose = [pi/2, 0.0, 0.0,
               -pi/2, 0.0, 0.0,
               -pi/2, 0.0, 0.0,
               pi/2, 0.0, 0.0]

dogFlatPose = [1.4, -pi/2, 0.2,
               -1.4, pi/2, 0.2,
               -1.8, -1.3, 0.2,
               1.8, 1.3, 0.2]

dogStandPose = [-0.9, -pi/2, 1.9, #square
               0.9, pi/2, 1.9,
               -4.0, -pi/2, 1.9,
               4.0, pi/2, 1.9]

dogWalkPoses = [
    #fl down fr up
    [0.0, -pi/2, 1.25,
    0.9, pi/2, 2.3,
    -4.0, -pi/2, 1.9,
    pi, pi/2, 1.25],

    #fr down bl up
    [-0.9, -pi/2, 1.9,
    0.45, pi/2, 1.8,
    -4.0, -pi/2, 2.3,
    3.8, pi/2, 1.7],

    #bl down br up
    [-0.9, -pi/2, 1.9,
    0.45, pi/2, 1.8,
    -pi, -pi/2, 1.25,
    4.0, pi/2, 2.3],

    #br down fl up
    [-0.9, -pi/2, 2.3,
    0.9, pi/2, 1.9,
    -4.0, -pi/2, 1.9,
    pi, pi/2, 1.25]

]
