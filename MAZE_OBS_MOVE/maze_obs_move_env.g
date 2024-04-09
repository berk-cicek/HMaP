world {}

wall (world){} 

bin_side (wall):{ X:"t(-0.2 0.2 0.68) d(0 0 0 1)" , shape:box, size: [0.8 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

bin_side2 (wall):{ X:"t(-0.2 0.6 0.68) d(0 0 0 1)" , shape:box, size: [0.8 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

bin_side3 (wall):{ X:"t(0.2 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

bin_side4 (wall):{ X:"t(-0.6 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

box (world){
  shape:box, Q: "t(0.0 0.4 .68) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[1 0 1], mass: .1, contact:1,friction: 0.001,
  joint:rigid
}

wpoint (box){
   shape:marker, Q: "t(-0.08 0 0.11) d(0 0 0 1)", size:[0.0015]
}

pawn {
  shape:box, X: "t(0.12 0.4 0.68) d(0 0 0 1)", size:[0.04 0.04 0.04 .04], color:[1 1 1], mass: 1, contact:1,
}

pawn_handle (pawn){
   shape:box, Q: "t(0 0 0.08) d(0 0 0 1)", size:[0.015 0.015 0.120 .015], color:[0 1 1], mass: .1, contact:1, friction: 100
}

cpoint (pawn_handle){
   shape:marker, Q: "t(0 0 0.02) d(0 0 0 1)", size:[0.0015]
}

cpoint2 (pawn_handle){
   shape:marker, Q: "t(0 0 0.1) d(0 0 0 1)", size:[0.0015]
}

obstacle_1 (world){
  shape:box, Q: "t(-0.4 0.4 .68) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[1 0 0], mass: .1, contact:1,
  joint:rigid
}

obstacle_2 (world){
  shape:box, Q: "t(-0.3 0.4 .68) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[0 1 0], mass: .1, contact:1,
  joint:rigid
}

obstacle_3 (world){
  shape:box, Q: "t(-0.2 0.4 .68) d(0 0 0 1)", size:[.04 .04 .04 .04], color:[0 0 1], mass: .1, contact:1,
  joint:rigid
}
