world {}

table (world){
    shape:ssBox, Q: "t(0 0. .00)", size:[4. 4. .1 .02], color:[.3 .3 .3], mass: .1,  contact:1
}

tunnel (world){
  Q: "t(0 0. .00)"
}

tunnel_top (tunnel):{ Q:"t(-0.3 0.25 0.165) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [1 0.5 0], mass: .1,contact:-1}
tunnel_side (tunnel):{ Q:"t(-0.3 0.19 0.08) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
tunnel_side2 (tunnel):{ Q:"t(-0.3 0.31 0.08) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
box (tunnel){
  shape:ssBox, Q: "t(-0.3 0.25 .09001) d(0 0 0 1)", size:[0.3 .04 .05 .04], color:[0 1 1], mass: .1, contact: 1,
  joint:trans3, limits: [-10,10, -10,10 ,-10,10 ]
}

contact_point(box){
  shape:marker, Q:"t(0 0 .025) d(0 0 0 1)", size: [0.1]
}

bin (world){
  Q: "t(0 0. .00)"
}
bin_side (bin):{ Q:"t(-0.7 -0.6 0.08) d(0 0 0 1)" , shape:box, size: [0.4 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
bin_side2 (bin):{ Q:"t(-0.7 -0.2 0.08) d(0 0 0 1)" , shape:box, size: [0.4 0.03 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
bin_side3 (bin):{ Q:"t(-0.5 -0.4 0.08) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}
bin_side4 (bin):{ Q:"t(-0.9 -0.4 0.08) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: .1,contact:-1}

