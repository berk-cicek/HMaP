world {}

pawn (world){
  shape:ssCylinder, Q: "t(0.12 0.4 0.66) d(0 0 0 1)", size:[.001 .03 .01], color:[1 1 1], mass: 1, contact:1,
}

pawn_handle (pawn){
   shape:box, Q: "t(0 0 0.07) d(0 0 0 1)", size:[0.015 0.015 0.120 .015], color:[0 1 1], mass: .1, contact:1, friction: 10
}

cpoint (pawn_handle){
   shape:marker, Q: "t(0 0 0.02) d(0 0 0 1)", size:[0.015]
}

cpoint2 (pawn_handle){
   shape:marker, Q: "t(0 0 0.03) d(0 0 0 1)", size:[0.015]
}
