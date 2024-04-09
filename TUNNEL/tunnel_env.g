world {}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .60)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact
}

lock_top (wall):{ X:"t(-0.9 0.25 0.765) d(0 0 0 1)" , shape:box, size: [0.9 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.9 0.19 0.68) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.9 0.31 0.68) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_top (wall):{ X:"t(-0.2 0.25 0.765) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_side (wall):{ X:"t(-0.2 0.19 0.68) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_side2 (wall):{ X:"t(-0.2 0.31 0.68) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

Include: <../../rai-robotModels/panda/panda_tunnel.g>
Edit panda_link0{ X: "t(0. -0.1 .65) d(90 0 0 1)" }

Edit panda_finger_joint1 { joint_active: False }

box (world){
  shape:ssBox, Q: "t(-0.64 0.25 .69) d(0 0 0 1)", size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:rigid
}

cam_5: {pose: [0, 0.25, 2.6, 0.0007963267107332633, 0.9999996829318346, 0.0, 0.0], shape: marker, size: [0.1], focalLength: 2, width: 640, height: 360, zRange: [0.5, 100] }


