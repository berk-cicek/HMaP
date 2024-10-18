world: {  },
ref: { pose: [0, 0, 3, 1, 0, 0, 0], shape: marker, size: [0.1] },
table(world): { shape: ssBox, size: [4, 4, 0.1, 0.02], color: [0.3, 0.3, 0.3], contact: 1, mass: 10, inertia: [13.3417, 13.3417, 26.6667] },

shelf(world): { shape: marker, size: [0.01] },
shelf_center(shelf): { rel: [0, -0.58, 0.5, -1, 0, 0, 0], shape: box, size: [0.7, 0.4, 0.03, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.0134083, 0.0409083, 0.0541667] },
shelf_back(shelf): { rel: [0, -0.79, 0.35, -1, 0, 0, 0], shape: box, size: [0.7, 0.03, 0.7, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.175283, 0.216042, 0.0409083] },
shelf_front(shelf): { rel: [0, -0.39, 0.25, -1, 0, 0, 0], shape: box, size: [0.7, 0.03, 0.5, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.175283, 0.216042, 0.0409083] },

shelf_book_1(shelf): { rel: [-0.29, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
#shelf_book_2(shelf): { rel: [-0.23, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
#shelf_book_3(shelf): { rel: [-0.17, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
shelf_book_4(shelf): { rel: [-0.11, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
shelf_book_5(shelf): { rel: [-0.05, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
shelf_book_6(shelf): { rel: [0.01, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
shelf_book_7(shelf): { rel: [0.07, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
shelf_book_8(shelf): { rel: [0.13, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },
shelf_book_9(shelf): { rel: [0.29, -0.65, 0.615, 1, 0, 0, 0], shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [1, 0, 0], contact: -1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },


box (world): { X:"t(0 0 0) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [0 1 1], mass: .5,contact:1,
    joint:free, limits: [-10, 10, -10, 10, 0.4, 0.8,-4,4,-4,4,-4,4,-4,4]
}

obstacle_0 (shelf): { X:"t(-0.2 -0.5 0.615) d(0 0 0 1)" , shape:box, size: [0.14 0.03 0.20 0.5], color: [1 1 0], mass: .1, contact:1,
  joint:rigid
}
