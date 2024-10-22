world {}

table(world): { shape: ssBox, size: [4, 4, 0.1, 0.02], color: [0.3, 0.3, 0.3], contact: 1, mass: 10, inertia: [13.3417, 13.3417, 26.6667] },

bin (table){} 
bin_side (bin):{ X:"t(0 -0.4 0.1) d(0 0 0 1)" , shape:box, size: [0.8 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
bin_side2 (bin):{ X:"t(0 0 0.1) d(0 0 0 1)" , shape:box, size: [0.8 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
bin_side3 (bin):{ X:"t(0.4 -0.2 0.1) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
bin_side4 (bin):{ X:"t(-0.4 -0.2 0.1) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

box (world){
  X: "t(0 0 0) d(0 0 0 1)", shape:box, size:[.08 .08 .08 .04], color:[1 0 1], mass: .1, contact:1,friction: 0.001,
  joint:free, limits: [-10,10,-10,10, 0, 0.1, -1,1, -1,1, -1,1, -1,1]
}

obstacle_0 (world){ shape:box, Q: "t(-0.3 -0.2 .08) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[1 0 0], mass: .1, contact:1, joint:rigid}
obstacle_1 (world){ shape:box, Q: "t(-0.26 -0.2 .08) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[0 1 0], mass: .1, contact:1, joint:rigid}
obstacle_2 (world){ shape:box, Q: "t(-0.22 -0.2 .08) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[0 0 1], mass: .1, contact:1, joint:rigid}