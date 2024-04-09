world {}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .60)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact
}

stick (world){
  shape:ssBox, Q: "t(0.34 0.25 .69) d(90 0 0 1)", size:[0.4 .02 .025 .04], color:[0 1 0], mass: .1, contact:1,
  joint:rigid
}

sphere (world){
  shape:sphere, Q: "t(0.5 0.25 .70) d(90 0 0 1)", size:[.05], color:[1 0 1], mass: .1, contact:1,
  joint:rigid
}

cube (world){
  shape:box, Q: "t(0.5 0.05 .70) d(90 0 0 1)", size:[0.08 .08 .08 .04], color:[0 0 1], mass: .1, contact:1,
  joint:rigid
}

lock_top (wall):{ X:"t(-0.3 0.25 0.765) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [1 0.5 0], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.3 0.19 0.68) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.3 0.31 0.68) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}



bin_side (wall):{ X:"t(-0.7 -0.6 0.68) d(0 0 0 1)" , shape:box, size: [0.4 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

bin_side2 (wall):{ X:"t(-0.7 -0.2 0.68) d(0 0 0 1)" , shape:box, size: [0.4 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

bin_side3 (wall):{ X:"t(-0.5 -0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

bin_side4 (wall):{ X:"t(-0.9 -0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

Include: <../../rai-robotModels/panda/panda_tunnel.g>
Edit panda_link0{ X: "t(0. -0.1 .65) d(90 0 0 1)" }

Edit panda_finger_joint1 { joint_active: False }

box (world){
  shape:ssBox, Q: "t(-0.3 0.25 .69) d(0 0 0 1)", size:[0.3 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:rigid
}

cam_5: {pose: [0, 0.25, 2.6, 0.0007963267107332633, 0.9999996829318346, 0.0, 0.0], shape: marker, size: [0.1], focalLength: 2, width: 640, height: 360, zRange: [0.5, 100] }


