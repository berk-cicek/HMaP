world {}

wall (world){} 

lock_top (wall):{ X:"t(-0.6 0.25 0.748) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.6 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.6 0.35 0.66) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock0_side (wall):{ X:"t(-0.4 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_side (wall):{ X:"t(-0.2 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_side2 (wall):{ X:"t(-0.2 0.35 0.66) d(0 0 0 1)" , shape:box, size: [0.1 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_top1 (wall):{ X:"t(-0.2 0.16 0.748) d(0 0 0 1)" , shape:box, size: [0.1 0.04 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock1_top2 (wall):{ X:"t(-0.2 0.34 0.748) d(0 0 0 1)" , shape:box, size: [0.1 0.05 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock02_side (wall):{ X:"t(-0.0 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock2_top (wall):{ X:"t(0.1 0.25 0.748) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock2_side (wall):{ X:"t(0.1 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

lock2_side2 (wall):{ X:"t(0.1 0.35 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

Include: <../../rai-robotModels/panda/panda_tunnel.g>

Edit panda_link0{ X: "t(-0.2 0.9 .59) d(-90 0 0 1)" }
#Edit panda_link0{ X: "t(0. -0.1 .65) d(90 0 0 1)" }

Edit panda_finger_joint1 { joint_active: False }

table (world){
    shape:ssBox, Q: "t(0 0. .54)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact
}

box (world){
  shape:ssBox, Q: "t(-0.32 0.25 .66) d(0 0 0 1)", size:[1.0 .1 .1 .04], color: [0.211 0.321 0.678], mass: .1, contact:1,
  joint:rigid
}

head (box){
  shape:ssBox, Q: "t(-0.05 0.07 .00) d(0 0 0 1)", size:[.02 .2 .01 .02], color:[0 1 1], mass: .1, contact:1
}

cam_5: {pose: [0, 0.25, 2, 0.0007963267107332633, 0.9999996829318346, 0.0, 0.0], shape: marker, size: [0.1], focalLength: 2, width: 640, height: 360, zRange: [0.5, 100] }
