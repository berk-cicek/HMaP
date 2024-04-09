world {}

box (world){
  shape:ssCylinder, Q: "t(0.0 0.4 .66) d(0 0 0 1)", size:[.001 .03 .01], color:[0 1 1], mass: .1, contact:1,friction: 0.001,
  joint:rigid
}

#wpoint (box){
#   shape:marker, Q: "t(-0.08 0 0.11) d(0 0 0 1)", size:[0.015]
#}
