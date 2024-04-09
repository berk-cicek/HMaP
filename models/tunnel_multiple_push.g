world {}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .0)", size:[4. 4. .1 .02], color:[0.3 0.3 0.3], contact
}

box (table){
  shape:ssBox, Q: "t(-0.64 0.25 .09) d(0 0 0 1)", size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:rigid
}

lock_top (wall):{ X:"t(-0.9 0.25 0.145) d(0 0 0 1)" , shape:box, size: [0.9 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.9 0.19 0.06) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.9 0.31 0.06) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_top (wall):{ X:"t(-0.2 0.25 0.145) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side (wall):{ X:"t(-0.2 0.19 0.06) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side2 (wall):{ X:"t(-0.2 0.31 0.06) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

Include: </home/bora/Desktop/LiRa/work4/rai-robotModels/panda/panda_tunnel.g>

Edit panda_link0{ X: "t(0. -0.1 .05) d(90 0 0 1)" }

Edit panda_finger_joint1 { joint_active: False }


final:{ X:"t(0.4 0.25 0.09) d(0 0 0 1)" , shape: marker, size: [.1]}
#middle:{ X:"t(0 0.25 0.7) d(0 0 0 1)" , shape: marker, size: [.01]}
middle:{ X:"t(0 0.25 0.09) d(0 0 0 1)" , shape: marker, size: [.1]}


cam_1: { pose: [0, -1.5, 1, 0.499998, -0.866027, 0, 0], shape: marker, size: [0.1], focalLength: 1.5, width: 640, height: 360, zRange: [0.5, 100] }
cam_2: { pose: [0, 1.5, 1, 0.499998, 0.866027, 0, 0], shape: marker, size: [0.1], focalLength: 1.5, width: 640, height: 360, zRange: [0.5, 100] }
cam_3: { pose: [1.5, 0, 1, 0.499998, 0, -0.866027, 0], shape: marker, size: [0.1], focalLength: 1.5, width: 640, height: 360, zRange: [0.5, 100] }
cam_4: { pose: [-1.5, 0, 1, 0.499998, 0, 0.866027, 0], shape: marker, size: [0.1], focalLength: 1.5, width: 640, height: 360, zRange: [0.5, 100] }

cam_5: {pose: [0, 0.25, 2, 0.0007963267107332633, 0.9999996829318346, 0.0, 0.0], shape: marker, size: [0.1], focalLength: 2, width: 640, height: 360, zRange: [0.5, 100] }