world {}

box (world){
  shape:ssCylinder, Q: "t(0.0 0.4 .66) d(0 0 0 1)", size:[.001 .03 .01], color:[0 1 1], mass: .1, contact:1,friction: 0.001,
  joint:rigid
}

pawn {
  shape:ssCylinder, X: "t(0.12 0.4 0.66) d(0 0 0 1)", size:[.001 .03 .01], color:[1 1 1], mass: 1, contact:1,
}

pawn_handle (pawn){
   shape:box, Q: "t(0 0 0.07) d(0 0 0 1)", size:[0.015 0.015 0.120 .015], color:[0 1 1], mass: .1, contact:1, friction: 100
}

cpoint (pawn_handle){
   shape:marker, Q: "t(0 0 0.02) d(0 0 0 1)", size:[0.0015]
}

cpoint2 (pawn_handle){
   shape:marker, Q: "t(0 0 0.03) d(0 0 0 1)", size:[0.0015]
}

wall (world){} 

bin_side (wall):{ X:"t(-0.1 0.1 0.68) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}

bin_side2 (wall):{ X:"t(-0.1 0.7 0.68) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}

bin_side3 (wall):{ X:"t(0.4 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}

bin_side4 (wall):{ X:"t(-0.6 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [1 0.5 0], mass: 10,contact:-1}

bin_side5 (wall):{ X:"t(-0.2 0.45 0.68) d(0 0 0 1)" , shape:box, size: [0.15 0.25 0.1 0.5], color: [1 0.5 0], mass: 10,contact:1}

#Include: <../rai-robotModels/scenarios/pandaSingle.g>