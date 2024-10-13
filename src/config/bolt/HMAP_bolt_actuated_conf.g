world {}

table (world){
    shape:ssBox, Q: "t(0 0. 0)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact: 1
}

lock (table){} 

lock_top (lock):{ X:"t(-0.6 0.25 0.208) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_side (lock):{ X:"t(-0.6 0.15 0.12) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_side2 (lock):{ X:"t(-0.6 0.35 0.12) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_0_side (lock):{ X:"t(-0.4 0.15 0.12) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_1_side (lock):{ X:"t(-0.2 0.15 0.12) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_1_side2 (lock):{ X:"t(-0.2 0.35 0.12) d(0 0 0 1)" , shape:box, size: [0.1 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_1_top1 (lock):{ X:"t(-0.2 0.16 0.208) d(0 0 0 1)" , shape:box, size: [0.1 0.04 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_1_top2 (lock):{ X:"t(-0.2 0.34 0.208) d(0 0 0 1)" , shape:box, size: [0.1 0.05 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_02_side (lock):{ X:"t(-0.0 0.15 0.12) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_2_top (lock):{ X:"t(0.1 0.25 0.208) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_2_side (lock):{ X:"t(0.1 0.15 0.12) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}
lock_2_side2 (lock):{ X:"t(0.1 0.35 0.12) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: .1,contact:-1}

box (world){
  X: "t(0 0 0) d(0 0 0 1)", shape:ssBox, size:[1.0 .02 .02 .04], color: [0.211, 0.321, 0.678], mass: .1, contact:1,
  joint:free, limits: [-10, 10, -10, 10, -10, 10, -10, 10, -10, 10, -10, 10, -10, 10]
}

head (box){
  shape:ssBox, Q: "t(-0.05 0.12 .00) d(0 0 0 1)", size:[.01 .3 .01 .02], color:[0 1 1], mass: .1, contact:1
}
