world: {  },
table(world): { shape: ssBox, size: [4, 4, 0.1, 0.02], color: [0.3, 0.3, 0.3], contact: 1, mass: 1, inertia: [1.33417, 1.33417, 2.66667] },
stick(world): { rel: [0.34, 0.25, 0.09, 0.707107, 0, 0, 0.707107], joint: rigid, shape: ssBox, size: [0.4, 0.08, 0.08, 0.04], color: [0, 1, 0], contact: 1, mass: 0.1, inertia: [0.000106667, 0.00138667, 0.00138667] },
sphere(world): { rel: [0.5, 0.25, 0.1, 0.707107, 0, 0, 0.707107], joint: rigid, shape: box, size: [0.08, 0.08, 0.08, 0.04], color: [1, 0, 1], contact: 1, mass: 0.1, inertia: [0.000106667, 0.000106667, 0.000106667] },
cube(world): { rel: [0.1, 0.5, 0.1, 0.707107, 0, 0, 0.707107], joint: rigid, shape: box, size: [0.08, 0.08, 0.08, 0.04], color: [0, 0, 1], contact: 1, mass: 0.1, inertia: [0.000106667, 0.000106667, 0.000106667] },
tunnel(world): { shape: marker, size: [0.01] },
tunnel_top(tunnel): { rel: [-0.3, 0.25, 0.165, 1, 0, 0, 0], shape: box, size: [0.3, 0.14, 0.03, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.037875, 0.0456667] },
tunnel_side(tunnel): { rel: [-0.3, 0.19, 0.08, 1, 0, 0, 0], shape: box, size: [0.3, 0.03, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.0456667, 0.037875] },
tunnel_side2(tunnel): { rel: [-0.3, 0.31, 0.08, 1, 0, 0, 0], shape: box, size: [0.3, 0.03, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.0456667, 0.037875] },
box(tunnel): { rel: [-0.3, 0.25, 0.09001, 1, 0, 0, 0], joint: rigid, shape: ssBox, size: [0.3, 0.08, 0.08, 0.04], color: [0, 1, 1], contact: 1, mass: 0.1, inertia: [0.000106667, 0.000803333, 0.000803333] },
bin(world): { shape: marker, size: [0.01] },
bin_side(bin): { rel: [-0.7, -0.6, 0.08, 1, 0, 0, 0], shape: box, size: [0.4, 0.03, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.0748333, 0.0670417] },
bin_side2(bin): { rel: [-0.7, -0.2, 0.08, 1, 0, 0, 0], shape: box, size: [0.4, 0.03, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.00854167, 0.0748333, 0.0670417] },
bin_side3(bin): { rel: [-0.5, -0.4, 0.08, 1, 0, 0, 0], shape: box, size: [0.03, 0.4, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.0748333, 0.00854167, 0.0670417] },
bin_side4(bin): { rel: [-0.9, -0.4, 0.08, 1, 0, 0, 0], shape: box, size: [0.03, 0.4, 0.14, 0.5], color: [1, 0.5, 0], contact: -1, mass: 5, inertia: [0.0748333, 0.00854167, 0.0670417] },
contact_point(box): { rel: [0, 0, 0.025, 1, 0, 0, 0], shape: marker, size: [0.1] },
cam_frame_0: { pose: [-0.4, 0.1, 2, 0.0007963, -1, 0, 0], shape: marker, size: [0.1] },
cam_frame_1: { pose: [0.4, 0.1, 2, 0.11685177,  -0.68908610,  -0.70502345,   0.12019962], shape: marker, size: [0.1] },
cam_frame_2: { pose: [-1.2, 0.1, 2, 0.12603066,  -0.70400444,   0.68787629,  -0.12377495], shape: marker, size: [0.1] },
r_panda_base: { multibody: True },
r_panda_link0(r_panda_base): { rel: [-1, 0.4, 0.05, 0.707388, 0, 0, 0.706825] },
r_panda_link0_0(r_panda_link0): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link0.ply>, visual: True },
r_panda_joint1_origin(r_panda_link0): { rel: [0, 0, 0.333, 1, 0, 0, 0] },
r_panda_joint1(r_panda_joint1_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
r_panda_link1(r_panda_joint1): {  },
r_panda_link1_0(r_panda_link1): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link1.ply>, visual: True },
r_panda_joint2_origin(r_panda_link1): { rel: [0, 0, 0, 0.707107, -0.707107, 0, 0] },
r_panda_joint2(r_panda_joint2_origin): { rel: [0, 0, 0, 0.877583, 0, 0, -0.479426], joint: hingeZ, limits: [-1.7628, 1.7628, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
r_panda_link2(r_panda_joint2): {  },
r_panda_link2_0(r_panda_link2): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link2.ply>, visual: True },
r_panda_joint3_origin(r_panda_link2): { rel: [0, -0.316, 0, 0.707107, 0.707107, 0, 0] },
r_panda_joint3(r_panda_joint3_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
r_panda_link3(r_panda_joint3): {  },
r_panda_link3_0(r_panda_link3): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link3.ply>, visual: True },
r_panda_joint4_origin(r_panda_link3): { rel: [0.0825, 0, 0, 0.707107, 0.707107, 0, 0] },
r_panda_joint4(r_panda_joint4_origin): { rel: [0, 0, 0, 0.540302, 0, 0, -0.841471], joint: hingeZ, limits: [-3.0718, -0.0698, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
r_panda_link4(r_panda_joint4): {  },
r_panda_link4_0(r_panda_link4): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link4.ply>, visual: True },
r_panda_joint5_origin(r_panda_link4): { rel: [-0.0825, 0.384, 0, 0.707107, -0.707107, 0, 0] },
r_panda_joint5(r_panda_joint5_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
r_panda_link5(r_panda_joint5): {  },
r_panda_link5_0(r_panda_link5): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link5.ply>, visual: True },
r_panda_joint6_origin(r_panda_link5): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0] },
r_panda_joint6(r_panda_joint6_origin): { rel: [0, 0, 0, 0.540302, 0, 0, 0.841471], joint: hingeZ, limits: [0.5, 3, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
r_panda_link6(r_panda_joint6): {  },
r_panda_link6_0(r_panda_link6): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link6.ply>, visual: True },
r_panda_joint7_origin(r_panda_link6): { rel: [0.088, 0, 0, 0.707107, 0.707107, 0, 0] },
r_panda_joint7(r_panda_joint7_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
r_panda_link7(r_panda_joint7): {  },
r_panda_link7_0(r_panda_link7): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link7.ply>, visual: True },
r_panda_joint8_origin(r_panda_link7): { rel: [0, 0, 0.107, 1, 0, 0, 0] },
r_panda_joint8(r_panda_joint8_origin): {  },
r_panda_link8(r_panda_joint8): {  },
r_panda_hand_joint_origin(r_panda_link8): { rel: [0, 0, 0, 0.92388, 0, 0, -0.382683] },
r_panda_hand_joint(r_panda_hand_joint_origin): {  },
r_panda_hand(r_panda_hand_joint): {  },
r_panda_hand_0(r_panda_hand): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/hand.ply>, visual: True },
r_panda_finger_joint1_origin(r_panda_hand): { rel: [0, 0, 0.0584, 1, 0, 0, 0] },
r_panda_finger_joint2_origin(r_panda_hand): { rel: [0, 0, 0.0584, 1, 0, 0, 0] },
r_panda_finger_joint1(r_panda_finger_joint1_origin): { rel: [0, 0.045, 0, 1, 0, 0, 0], joint: transY, limits: [0, 0.04, 0.2, -1, 20, 0.2, -1, 20], ctrl_limits: [0.2, -1, 20] },
r_panda_finger_joint2(r_panda_finger_joint2_origin): { rel: [-0, -0.045, -0, -1, 0, 0, 0], joint: transY, joint_scale: -1, limits: [0, 0.04, 0.2, -1, 20, 0.2, -1, 20], mimic: "r_panda_finger_joint1", ctrl_limits: [0.2, -1, 20] },
r_panda_leftfinger(r_panda_finger_joint1): {  },
r_panda_rightfinger(r_panda_finger_joint2): {  },
r_panda_leftfinger_0(r_panda_leftfinger): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/finger.ply>, visual: True },
r_panda_rightfinger_0(r_panda_rightfinger): { rel: [0, 0, 0, -1.03412e-13, 0, 0, 1], shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/finger.ply>, visual: True },
r_panda_coll1(r_panda_joint1): { rel: [0, 0, -0.15, 1, 0, 0, 0], shape: capsule, size: [0.2, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
r_panda_coll3(r_panda_joint3): { rel: [0, 0, -0.15, 1, 0, 0, 0], shape: capsule, size: [0.2, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
r_panda_coll5(r_panda_joint5): { rel: [0, 0.02, -0.2, 1, 0, 0, 0], shape: capsule, size: [0.22, 0.09], color: [1, 1, 1, 0.2], contact: -2 },
r_panda_coll2(r_panda_joint2): { shape: capsule, size: [0.12, 0.12], color: [1, 1, 1, 0.2], contact: -2 },
r_panda_coll4(r_panda_joint4): { shape: capsule, size: [0.12, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
r_panda_coll6(r_panda_joint6): { rel: [0, 0, -0.04, 1, 0, 0, 0], shape: capsule, size: [0.1, 0.07], color: [1, 1, 1, 0.2], contact: -2 },
r_panda_coll7(r_panda_joint7): { rel: [0, 0, 0.01, 1, 0, 0, 0], shape: capsule, size: [0.1, 0.07], color: [1, 1, 1, 0.2], contact: -2 },
r_l_gripper(r_panda_joint7): { rel: [-2.69422e-17, 0, 0.22, 2.34326e-17, 0.92388, 0.382683, 5.65713e-17],
shape: marker,
size: [0.03],
color: [0.9, 0.9, 0.9],
logical: { is_gripper: True } },
r_palm(r_panda_hand_joint): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0], shape: capsule, size: [0.14, 0.07], color: [1, 1, 1, 0.2], contact: -3 },
r_finger1(r_panda_finger_joint1): { rel: [0, 0.028, 0.035, 1, 0, 0, 0], shape: capsule, size: [0.02, 0.03], color: [1, 1, 1, 0.2], contact: -2 },
r_finger2(r_panda_finger_joint2): { rel: [0, -0.028, 0.035, 1, 0, 0, 0], shape: capsule, size: [0.02, 0.03], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_base: { multibody: True },
l_panda_link0(l_panda_base): { rel: [0.3, -0.1, 0.05, 0.707388, 0, 0, 0.706825] },
l_panda_link0_0(l_panda_link0): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link0.ply>, visual: True },
l_panda_joint1_origin(l_panda_link0): { rel: [0, 0, 0.333, 1, 0, 0, 0] },
l_panda_joint1(l_panda_joint1_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link1(l_panda_joint1): {  },
l_panda_link1_0(l_panda_link1): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link1.ply>, visual: True },
l_panda_joint2_origin(l_panda_link1): { rel: [0, 0, 0, 0.707107, -0.707107, 0, 0] },
l_panda_joint2(l_panda_joint2_origin): { rel: [0, 0, 0, 0.877583, 0, 0, -0.479426], joint: hingeZ, limits: [-1.7628, 1.7628, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link2(l_panda_joint2): {  },
l_panda_link2_0(l_panda_link2): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link2.ply>, visual: True },
l_panda_joint3_origin(l_panda_link2): { rel: [0, -0.316, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint3(l_panda_joint3_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link3(l_panda_joint3): {  },
l_panda_link3_0(l_panda_link3): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link3.ply>, visual: True },
l_panda_joint4_origin(l_panda_link3): { rel: [0.0825, 0, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint4(l_panda_joint4_origin): { rel: [0, 0, 0, 0.540302, 0, 0, -0.841471], joint: hingeZ, limits: [-3.0718, -0.0698, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link4(l_panda_joint4): {  },
l_panda_link4_0(l_panda_link4): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link4.ply>, visual: True },
l_panda_joint5_origin(l_panda_link4): { rel: [-0.0825, 0.384, 0, 0.707107, -0.707107, 0, 0] },
l_panda_joint5(l_panda_joint5_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
l_panda_link5(l_panda_joint5): {  },
l_panda_link5_0(l_panda_link5): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link5.ply>, visual: True },
l_panda_joint6_origin(l_panda_link5): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint6(l_panda_joint6_origin): { rel: [0, 0, 0, 0.540302, 0, 0, 0.841471], joint: hingeZ, limits: [0.5, 3, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
l_panda_link6(l_panda_joint6): {  },
l_panda_link6_0(l_panda_link6): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link6.ply>, visual: True },
l_panda_joint7_origin(l_panda_link6): { rel: [0.088, 0, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint7(l_panda_joint7_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
l_panda_link7(l_panda_joint7): {  },
l_panda_link7_0(l_panda_link7): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/link7.ply>, visual: True },
l_panda_joint8_origin(l_panda_link7): { rel: [0, 0, 0.107, 1, 0, 0, 0] },
l_panda_joint8(l_panda_joint8_origin): {  },
l_panda_link8(l_panda_joint8): {  },
l_panda_hand_joint_origin(l_panda_link8): { rel: [0, 0, 0, 0.92388, 0, 0, -0.382683] },
l_panda_hand_joint(l_panda_hand_joint_origin): {  },
l_panda_hand(l_panda_hand_joint): {  },
l_panda_hand_0(l_panda_hand): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/hand.ply>, visual: True },
l_panda_finger_joint1_origin(l_panda_hand): { rel: [0, 0, 0.0584, 1, 0, 0, 0] },
l_panda_finger_joint2_origin(l_panda_hand): { rel: [0, 0, 0.0584, 1, 0, 0, 0] },
l_panda_finger_joint1(l_panda_finger_joint1_origin): { rel: [0, 0.045, 0, 1, 0, 0, 0], joint: transY, limits: [0, 0.04, 0.2, -1, 20, 0.2, -1, 20], ctrl_limits: [0.2, -1, 20] },
l_panda_finger_joint2(l_panda_finger_joint2_origin): { rel: [-0, -0.045, -0, -1, 0, 0, 0], joint: transY, joint_scale: -1, limits: [0, 0.04, 0.2, -1, 20, 0.2, -1, 20], mimic: "l_panda_finger_joint1", ctrl_limits: [0.2, -1, 20] },
l_panda_leftfinger(l_panda_finger_joint1): {  },
l_panda_rightfinger(l_panda_finger_joint2): {  },
l_panda_leftfinger_0(l_panda_leftfinger): { shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/finger.ply>, visual: True },
l_panda_rightfinger_0(l_panda_rightfinger): { rel: [0, 0, 0, -1.03412e-13, 0, 0, 1], shape: mesh, mesh: <../../../rai-robotModels/panda/franka_description/meshes/visual/finger.ply>, visual: True },
l_panda_coll1(l_panda_joint1): { rel: [0, 0, -0.15, 1, 0, 0, 0], shape: capsule, size: [0.2, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll3(l_panda_joint3): { rel: [0, 0, -0.15, 1, 0, 0, 0], shape: capsule, size: [0.2, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll5(l_panda_joint5): { rel: [0, 0.02, -0.2, 1, 0, 0, 0], shape: capsule, size: [0.22, 0.09], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll2(l_panda_joint2): { shape: capsule, size: [0.12, 0.12], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll4(l_panda_joint4): { shape: capsule, size: [0.12, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll6(l_panda_joint6): { rel: [0, 0, -0.04, 1, 0, 0, 0], shape: capsule, size: [0.1, 0.07], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll7(l_panda_joint7): { rel: [0, 0, 0.01, 1, 0, 0, 0], shape: capsule, size: [0.1, 0.07], color: [1, 1, 1, 0.2], contact: -2 },
l_l_gripper(l_panda_joint7): { rel: [-2.69422e-17, 0, 0.22, 2.34326e-17, 0.92388, 0.382683, 5.65713e-17],
shape: marker,
size: [0.03],
color: [0.9, 0.9, 0.9],
logical: { is_gripper: True } },
l_palm(l_panda_hand_joint): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0], shape: capsule, size: [0.14, 0.07], color: [1, 1, 1, 0.2], contact: -3 },
l_finger1(l_panda_finger_joint1): { rel: [0, 0.028, 0.035, 1, 0, 0, 0], shape: capsule, size: [0.02, 0.03], color: [1, 1, 1, 0.2], contact: -2 },
l_finger2(l_panda_finger_joint2): { rel: [0, -0.028, 0.035, 1, 0, 0, 0], shape: capsule, size: [0.02, 0.03], color: [1, 1, 1, 0.2], contact: -2 }