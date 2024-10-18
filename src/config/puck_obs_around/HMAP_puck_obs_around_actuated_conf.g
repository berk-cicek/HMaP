world {}

table(world): { shape: ssBox, size: [4, 4, 0.1, 0.02], color: [0.3, 0.3, 0.3], contact: 1, mass: 10, inertia: [13.3417, 13.3417, 26.6667] },

bin (table){} 
bin_side (bin):{ X:"t(-0.1 -0.6 0.1) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}
bin_side2 (bin):{ X:"t(-0.1 0.0 0.1) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}
bin_side3 (bin):{ X:"t(0.4 -0.3 0.1) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}
bin_side4 (bin):{ X:"t(-0.6 -0.3 0.1) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}
bin_obs (bin)  :{ X:"t(-0.2 -0.20 0.1) d(0 0 0 1)" , shape:box, size: [0.15 0.25 0.1 0.5], color: [1 0.5 0], mass: 10,contact:1}

box (world){
  shape:ssCylinder, Q: "t(0 0 0) d(0 0 0 1)", size:[.001 .03 .01], color:[0 1 1], mass: .1, contact:1,friction: 0.001,
  joint:free, limits: [-10,10,-10,10, 0, 0.1, -1,1, -1,1, -1,1, -1,1]
}


