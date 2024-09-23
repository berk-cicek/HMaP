import os
import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
import random
import time
import torch

# Load the environment
environment = "BOLT/bolt_env.g"
config = ry.Config()
config.addFile(environment)
config.view()

# Initialize Action/Path Tree

action_tree = []

# Collect Frame States (it goes incrementally according to t. First frame is initial Config)
frame_list = []

# Goal_Pose and Contact_Point list
final_pose_list = []
contact_point_list = []

box = config.frame("box")

path_list = [[config.getJointState()]]
object_list = [[*box.getPosition(), *box.getQuaternion()]]


# Action Hyperparameters
delta = 0.2  # Duration of the action (Such as distance between nodes)
phase = 40  # Komo stepsPerPhase
trial = 8  # Number of action trials
max_iter = 40  # Max iteration number of main loop (=WP count) (=len(path))
rrt_extend_length = 0.1  # For RRT (not used right now)
threshold = 0.6  # Threshold of goal pose

goal_pos = np.array([0, 0.25, 0.12, 1, 0, 0, 0])


# Define RRT
def RRT(qF, rrt_extend_length=0.04):
    C2 = ry.Config()
    C2.addFile("BOLT/bolt_actuated_env.g")
    q0 = C2.getJointState()
    print("Start State: ", q0, " Final State: ", qF)
    # Create an RRT path finder
    rrt = ry.PathFinder()
    rrt.setProblem(C2, [q0], [qF])

    isFeas = False
    path = None

    i = 0
    # Find an RRT path
    while i != 50 and (not isFeas):
        rrt = ry.PathFinder()
        rrt.setProblem(C2, [q0], [qF])
        ret = rrt.solve()
        isFeas = ret.feasible
        path = ret.x
        i += 1

    ## Smooth the path (optimization) ##

    # Create a KOMO for post-optimization with acceleration constraints
    num_time_steps = len(path)

    komo2 = ry.KOMO(C2, num_time_steps, 1, 2, True)
    # Add an acceleration constraint objective to the new KOMO object
    komo2.addControlObjective([], 2, 1e1)
    komo2.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e2])
    # Set the waypoints for the KOMO
    komo2.initWithPath_qOrg(path)

    ret = (
        ry.NLP_Solver()
        .setProblem(komo2.nlp())
        .setOptions(stopTolerance=1e-2, verbose=0)
        .solve()
    )
    print(ret)

    path = komo2.getPath()

    for i in range(0, path.shape[0]):
        C2.setJointState(path[i])
        C2.view()
        time.sleep(0.02)
        path[i] = path[i] + [0, 0, 0.55, 0, 0, 0, 0]

    # print(path)
    return path


#### Use Vision/ Point cloud for CP estimation ####
def getCameraView(C, camName, filter=10, plot=False):
    """
    Given the camera name, the function returns the cloud point obtained by that camera. Optionally a filter can be given which adjusts the amount of
    point clouds (default is reduction by 10). If plot is True the rgb and depth image from the camera can be seen.
    """
    cam = ry.CameraView(C)
    cam.setCamera(camName)
    rgb, depth = cam.computeImageAndDepth(C)

    # Filtering the object
    mask = np.zeros(rgb[:, :, 1].shape)
    red = rgb[:, :, 0]
    green = rgb[:, :, 1]
    blue = rgb[:, :, 2]

    (x, y) = mask.shape
    for i in range(0, x):
        for j in range(0, y):
            if green[i][j] > 180 and blue[i][j] > 180 and red[i][j] < 50:
                mask[i][j] = 1

    newDepth = np.where(mask, depth, np.nan)
    pcl = ry.depthImage2PointCloud(newDepth, cam.getFxycxy())

    # Removing the other points
    cloud = []
    (x, y) = depth.shape
    for i in range(0, x):
        for j in range(0, y):
            if mask[i][j] and j % filter == 0:
                cloud.append(pcl[i][j])
    cloud = np.asarray(cloud)

    if plot:
        fig = plt.figure()
        fig.add_subplot(1, 2, 1)
        plt.imshow(rgb)
        fig.add_subplot(1, 2, 2)
        plt.imshow(depth)
        plt.show()

    return [cam, cloud]


prev_c = []


def findGraspPoint(C, c):
    global prev_c
    if c.shape == 0:
        print("Couldn't find point cloud")
        c = prev_c

    cam = C.frame("cam_5")
    grasp_point = cam.getPosition() - np.array(
        c[random.randint(0, (c.shape[0] - 1)), :]
    )
    grasp_point[0] *= -1
    prev_c = c
    return grasp_point


def isContactFeasible(C, t, contact_point):
    S = ry.Skeleton()
    S.enableAccumulatedCollisions(True)
    S.addEntry([t, t], ry.SY.touch, ["gripper", contact_point])
    komo = S.getKomo_path(
        C,
        stepsPerPhase=30,
        accScale=1e0,
        lenScale=1e-2,
        homingScale=1e-1,
        collScale=1e3,
    )
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    print(ret)
    return ret.feasible


def addMarker(C, pos, name, quat=[1, 0, 0, 0]):
    print(f"CONTACT POINT:  {pos[:]}")
    marker = (
        C.addFrame(name, parent="head")
        .setShape(ry.ST.ssBox, size=[0, 0, 0, 0.01])
        .setPosition([*pos[:2], pos[2]])
        .setQuaternion(quat)
    )
    return marker


def addPoint(C, pos, name, parent, color):
    point = (
        C.addFrame(name, parent=parent)
        .setShape(ry.ST.sphere, size=[0, 0, 0, 0.02])
        .setPosition(pos)
        .setColor(color)
        .setContact(0)
    )
    C.view()
    return point


def addPoint2(C, pos, name, parent, color):
    pos = C.frame("head").getPosition()
    point = (
        C.addFrame(name, parent=parent)
        .setShape(ry.ST.sphere, size=[0, 0, 0, 0.02])
        .setPosition(pos)
        .setColor(color)
        .setContact(0)
    )
    C.view()
    return point


def convertCPrelative(C, new_name="rel_contact_point", name="contact_point"):
    relativeCP = config.getFrame(name)
    rel_CP_pos = relativeCP.getRelativePosition()
    relCP = C.addFrame(new_name, parent="head").setRelativePosition(rel_CP_pos)


# Generate Contact point
def CPgeneration(C):
    [p1, c] = getCameraView(C, "cam_5", 10, False)
    grasp_point = findGraspPoint(C, c)
    print(grasp_point)
    addMarker(C, grasp_point, "contact_point")
    convertCPrelative(C, "rel_contact_point")
    # marker = addMarker(C, grasp_point, "contact_point",  [0.7038453156522361, 0.0, 0.0, 0.7103532724176078])
    itr = 0
    while itr != 10 and (not isContactFeasible(C, 1, "rel_contact_point")):
        itr += 1
        C.delFrame("contact_point")
        grasp_point = findGraspPoint(C, c)
        marker = addMarker(C, grasp_point, "contact_point")
        convertCPrelative(C, "rel_contact_point", "contact_point")
        # marker = addMarker(C, grasp_point, "contact_point", 0.1, [0.7038453156522361, 0.0, 0.0, 0.7103532724176078])
        print(f"This Contact is feasible (found in {itr} iteration)")
    addPoint(C, grasp_point, "grasp_conint", "head", [1, 1, 0])


# Push Action
def push(contact_point, final_pose, t=1):

    skeleton = ry.Skeleton()

    skeleton.addEntry([t, -1], ry.SY.stable, ["gripper", "box"])
    # skeleton.addEntry([t, -1], ry.SY.contactStick,  ["gripper", "box"])
    # skeleton.addEntry([t, -1], ry.SY.touch,  ["gripper", "head"])  #Check this part for optimal motion
    skeleton.addEntry([t + delta, t + delta], ry.SY.poseEq, ["box", final_pose])
    # skeleton.addEntry([t, -1], ry.SY.quasiStaticOn, ["table", "box"])
    skeleton.enableAccumulatedCollisions(True)

    komo = skeleton.getKomo_path(
        config,
        stepsPerPhase=phase,
        accScale=1e0,
        lenScale=1e-2,
        homingScale=1e-3,
        collScale=1e1,
    )
    komo.addObjective(
        [t, -1],
        ry.FS.positionDiff,
        ["gripper", contact_point],
        ry.OT.sos,
        target=[0.0],
        scale=[1e2],
    )
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()

    print(ret)
    return komo, ret


# WP feasibility check
def tryWP(gen_wp, i):
    print(f"Generated WP is: {gen_wp}")

    # Create as frame
    wpx = config.addFrame(name=f"wp{i}")
    wpx.setPosition(gen_wp[0:3])  # Generate WP Position
    wpx.setQuaternion(gen_wp[3:])
    addPoint2(
        config, [0, 0, 0], "point" + str(random.randint(0, 10000)), "world", [1, 0, 0]
    )
    return gen_wp


path = RRT(goal_pos, rrt_extend_length)

print(path)

# Set t=1 for first action
if len(action_tree) == 0:  # USELESS NOW, Can be removed
    t = 1
else:
    t = 0

i = 0
CPgeneration(config)  # Initial CP generator, it uses path[0] (first WP)
convertCPrelative(config, "contact_point")
gen_cp = "rel_contact_point"  # This will come from CP creator
gen_wp = np.zeros(np.shape(goal_pos))
while i != max_iter and not np.linalg.norm(gen_wp - goal_pos) < threshold:
    print(f"Step: {i+1}")
    fail = 0
    for n in range(trial):
        # gen_wp = WPgeneration() #Such as Extent function of RRT
        gen_wp = path[i]
        tryWP(gen_wp, i)

        if n >= 2:
            CPgeneration(config)  # This will become actual CP generator
            convertCPrelative(config, "contact_point")
            # gen_cp= "cp2" #This will become the result of new CPgeneration()
            gen_cp = "rel_contact_point"

        # Test Waypoint:#
        # Parameter Inıtialization
        end_pose = f"wp{i}"
        contact_point = gen_cp

        if i == 0:  # There is a problem when t is not always equal to 1
            t = 1
        elif n >= 2:
            t = 0.2
        else:
            t = 0

        # Solve Push Action with KOMO
        komo, ret = push(contact_point, end_pose, t)
        # Visualize
        # config.computeCollisions()
        # print(f"COLLISIONS: {config.getCollisions(belowMargin=-0.000001)}")
        # komo.view(True, "waypoints solution")
        # komo.view_play(True, 0.5)
        # komo.report(False, plotOverTime=True)

        if ret.eq <= 0.8:

            print("Action is feasible")
            # Set Final Configuration of current action
            waypoints = komo.getPath_qAll()
            Frame_state = komo.getFrameState(len(waypoints) - 1)

            frame_list.append(Frame_state)

            path_list.append(komo.getPath())
            object_list.append([*box.getPosition(), *box.getQuaternion()])

            final_pose_list.append(gen_wp)
            contact_point_list.append(gen_cp)
            config.setFrameState(Frame_state)
            config.view()
            break  # Continue to next WP in the path

        else:
            # config.delFrame("point" + str(i))
            print(f"Infeasible Action, Generate new WP,CP pair: {n}")
            feasibility = False
            fail += 1

    i += 1

    if fail != (trial):
        continue
    else:
        # Generate new WP list (path) in this case
        # WPgeneration()
        print("New path is calculated")
        print(
            "Try to use smaller 'extend_length' or higher 'phase'. Play with Hyperparameters"
        )
        print("Action Failed")
        break

if np.linalg.norm(gen_wp - goal_pos) < threshold:
    print("TRAJECTORY FOUND!!!!")

### Visualization ###
environment = "BOLT/bolt_env.g"
vis_config = ry.Config()
vis_config.addFile(environment)
bo = vis_config.frame("box")
vis_config.view()

for i, k in zip(path_list, object_list):
    bo.setPosition(k[0:3])
    bo.setQuaternion(k[3:])
    for j in i:
        vis_config.setJointState(j)

        vis_config.view()
        time.sleep(0.01)

for i in config.getFrameNames():
    if "panda" in i or i in ["gripper", "palm", "finger1", "finger2"]:
        config.delFrame(i)
config.view()
