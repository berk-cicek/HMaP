world {}
ref { pose: [0.0, 0.0, 3, 1, 0, 0, 0], shape: marker, size: [0.1]}

table (world){
    shape:ssBox, Q: "t(0 0. .00)", size:[4. 4. .1 .02], color:[.3 .3 .3], mass: 10, contact:1
}

shelf_side_left (table):{ X:"t(-0.35 -0.58 0.725) d(0 0 0 1)" , shape:box, size: [0.03 0.40 1.45 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_side_right (table):{ X:"t(0.35 -0.58 0.725) d(0 0 0 1)" , shape:box, size: [0.03 0.40 1.45 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_bottom (table):{ X:"t(0 -0.58 0.075) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_top (table):{ X:"t(0.0 -0.58 1.45) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_center_mid (table):{ X:"t(0.0 -0.58 1.15) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_center_low (table):{ X:"t(0.0 -0.58 0.8) d(0 0 0 1)" , shape:box, size: [0.70 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}
shelf_back (table):{ X:"t(0.0 -0.7 0.725) d(0 0 0 1)" , shape:box, size: [0.70 0.03 1.45 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

book_1 (shelf_center_mid): { X:"t(-0.29 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_2 (shelf_center_mid): { X:"t(-0.23 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_3 (shelf_center_mid): { X:"t(-0.17 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_4 (shelf_center_mid): { X:"t(-0.11 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_5 (shelf_center_mid): { X:"t(-0.05 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_6 (shelf_center_mid): { X:"t(0.01 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_7 (shelf_center_mid): { X:"t(0.07 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_8 (shelf_center_mid): { X:"t(0.13 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_9 (shelf_center_mid): { X:"t(0.29 -0.58 1.265) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}

box (world): { X:"t(0 -0.58 0.91) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [0 1 1], mass: .5,contact:1,
    joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]
}

obstacle_0 (world): { X:"t(-0.05 -0.4 0.91) d(0 0 0 1)" , shape:box, size: [0.14 0.03 0.20 0.5], color: [1 1 0], mass: .1, contact:1,
  joint:rigid
}
