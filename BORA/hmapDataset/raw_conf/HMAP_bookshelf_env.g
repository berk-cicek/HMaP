world {}
ref { pose: [0.0, 0.0, 3, 1, 0, 0, 0], shape: marker, size: [0.1]}

table (world){
    shape:ssBox, Q: "t(0 0. .00)", size:[4. 4. .1 .02], color:[.3 .3 .3], mass: 10, contact:1
}

shelf_side_left (table):{ X:"t(-0.35 -0.5 0.725) d(0 0 0 1)" , shape:box, size: [0.03 0.40 1.45 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_side_right (table):{ X:"t(0.35 -0.5 0.725) d(0 0 0 1)" , shape:box, size: [0.03 0.40 1.45 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_bottom (table):{ X:"t(0 -0.5 0.075) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_top (table):{ X:"t(0.0 -0.5 1.45) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_center_mid (table):{ X:"t(0.0 -0.5 1.15) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_center_low (table):{ X:"t(0.0 -0.5 0.8) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_back (table):{ X:"t(0.0 -0.7 0.725) d(0 0 0 1)" , shape:box, size: [0.70 0.03 1.45 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

book_1 (shelf_center_mid): { X:"t(-0.29 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_2 (shelf_center_mid): { X:"t(-0.23 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_3 (shelf_center_mid): { X:"t(-0.17 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_4 (shelf_center_mid): { X:"t(-0.11 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_5 (shelf_center_mid): { X:"t(-0.05 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_6 (shelf_center_mid): { X:"t(0.01 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_7 (shelf_center_mid): { X:"t(0.07 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_7 (shelf_center_mid): { X:"t(0.13 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_9 (shelf_center_mid): { X:"t(0.29 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}

box (world): { X:"t(0 -0.58 0.91) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [0 1 1], mass: .1,contact:1,
  joint:rigid
}

obstacle_0 (world): { X:"t(-0.05 -0.4 0.91) d(0 0 0 1)" , shape:box, size: [0.14 0.03 0.20 0.5], color: [1 1 0], mass: .1, contact:1,
  joint:rigid
}

stick (world){
  shape:ssBox, Q: "t(0.6 0.05 .1) d(90 0 0 1)", size:[0.3 .02 .025 .02], color:[1 0 1], mass: .1, contact:1,
  joint:rigid
}

cam_frame_0(world): { pose: [0.0, 0.6, 1.2, 0, 0, -0.7071068, 0.7071068], shape: marker, size: [0.1]}
cam_frame_0_base(world): { pose: [0.0, 0.6, 1.2, 0, 0, -0.7071068, 0.7071068], shape: marker, size: [0.01]}
cam_frame_1(world): { pose: [0.0, 0.6, 1.4, 0, 0, 0.8320503, -0.5547002], shape: marker, size: [0.1]}
cam_frame_1_base(world): { pose: [0.0, 0.6, 1.4, 0, 0, -0.7071068, 0.7071068], shape: marker, size: [0.01]}

#cam_main: {pose: [-0.5, -1, 1.6, 0.5403023, -0.841471, 0, 0], shape: marker, size: [0.1], focalLength: 0.5, width: 640, height: 360, zRange: [0.5, 100] }