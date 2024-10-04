world {}

table (world){
    shape:ssBox, Q: "t(0 0. .00)", size:[4. 4. .1 .02], color:[.3 .3 .3], mass: .1,  contact:1
}

tunnel (world){
  Q: "t(0 0. .00)"
}

tunnel_top(tunnel): { rel: [-0.15, 0.25, 0.165, 1, 0, 0, 0], shape: box, size: [0.3, 0.14, 0.03, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.037875, 0.0456667] },
tunnel_side(tunnel): { rel: [-0.15, 0.19, 0.08, 1, 0, 0, 0], shape: box, size: [0.3, 0.03, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.0456667, 0.037875] },
tunnel_side2(tunnel): { rel: [-0.15, 0.31, 0.08, 1, 0, 0, 0], shape: box, size: [0.3, 0.03, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.0456667, 0.037875] },

box (world){
  shape:ssBox, Q: "t(0 0 0) d(0 0 0 1)", size:[0.3 .04 .05 .04], color:[0 1 1], mass: .1, contact: 1,
  joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]
}

stick(world): { rel: [0.34, 0.25, 0.09001, 0.707107, 0, 0, 0.707107], joint: rigid, shape: ssBox, size: [0.4, 0.08, 0.08, 0.04], color: [0, 1, 0], contact: 1, mass: 0.1, inertia: [0.000106667, 0.00138667, 0.00138667] },
sphere(world): { rel: [0.5, 0.25, 0.1, 0.707107, 0, 0, 0.707107], joint: rigid, shape: box, size: [0.08, 0.08, 0.08, 0.04], color: [1, 0, 1], contact: 1, mass: 0.1, inertia: [0.000106667, 0.000106667, 0.000106667] },
cube(world): { rel: [0.1, 0.5, 0.1, 0.707107, 0, 0, 0.707107], joint: rigid, shape: box, size: [0.08, 0.08, 0.08, 0.04], color: [0, 0, 1], contact: 1, mass: 0.1, inertia: [0.000106667, 0.000106667, 0.000106667] },

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

