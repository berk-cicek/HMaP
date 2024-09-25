#include <BotOp/bot.h>
#include <Control/ShortPathMPC.h>
#include <Control/timingOpt.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/manipTools.h>
#include <KOMO/pathTools.h>
#include <KOMO/skeleton.h>
#include <Kin/viewer.h>
#include <Kin/F_forces.h>
#include <Kin/forceExchange.h>
#include <OptiTrack/optitrack.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <random>
#include <PathAlgos/ConfigurationProblem.h>
#include <PathAlgos/RRT_PathFinder.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Geo/depth2PointCloud.h>
#include <string>
#include <Kin/cameraview.h>
#include <Kin/frame.h>


/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int img_count = 0;
rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat = {0.7, 0, 0, 0.7});
bool RRT(rai::Configuration& C2, const arr& qF, arr& path, bool view = true, double rrt_extend_length = 0.04);
arr getCameraView(rai::Configuration& C, const std::string& cam_name, const std::string& target, const double filter = 0.5);
arr candidateContactPoint(rai::Configuration& C, const arr& pts, const int iter, bool isTransform = true);
const std::string generateContactPoint(rai::Configuration& C, const bool is_aval_l, const bool is_aval_r, const arr qHome, const std::string& target, const std::string& waypoint, const double filter, const double calib, std::string& contact_point, std::string& tool);
double calibSkeleton(rai::Configuration& C);
std::shared_ptr<SolverReturn> homeSkeleton(rai::Configuration& C, const std::string& l_gripper, const std::string& r_gripper, const std::string& l_gripper_home, const std::string& r_gripper_home);
std::shared_ptr<SolverReturn> moveSkeleton(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& tool, const std::string& contact_point, const std::string& waypoint, bool& is_aval_l, bool& is_aval_r, const double threshold, const double calib, const bool isTrial = false);
double toolSkeleton(rai::Configuration& C, const std::string& tool, const std::string& target, const std::string& gripper, const std::string& final_pose, const bool isTrial = false);
void toolSelection(rai::Configuration& C, const std::vector<std::string>& tool_list, const std::vector<std::string>&  gripper_list, const std::string waypoint, const std::string target, bool& is_aval_l, bool& is_aval_r, std::string& gripper_out, std::string& tool_out);
std::string useTool(rai::Configuration& C, const arr path_point, const std::vector<std::string>& tool_list, const std::vector<std::string>&  gripper_list, const std::string waypoint, const std::string target, bool& is_aval_l, bool& is_aval_r);

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    rai::initCmdLine(argc, argv);
    rnd.seed(0);

    // Add parameters
    rai::params()->add<double>("botsim/verbose", 0.0);
    rai::params()->add<double>("physx/motorKp", 10000.0);
    rai::params()->add<double>("physx/motorKd", 1000.0);
    rai::params()->add<rai::String>("botsim/engine", "physx");
    rai::params()->add<bool>("physx/multibody", true);

    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("HMAP_bookshelf_env.g");  

    rai::Configuration C2;
    C2.addFile("HMAP_bookshelf_actuated_env.g");  
    
    C.addFile(rai::raiPath("../rai-robotModels/panda/panda_tunnel.g"), "r_");
    C.addFile(rai::raiPath("../rai-robotModels/panda/panda_tunnel.g"), "l_");
    C.getFrame("r_panda_link0")->setPose(rai::Transformation(rai::Vector(-0.20, 0.1, .05), rai::Quaternion(0.7073883, 0., 0., -0.7068252)));
    C.getFrame("l_panda_link0")->setPose(rai::Transformation(rai::Vector(0.4, 0.1, .05), rai::Quaternion(0.7073883, 0., 0., -0.7068252)));
    C.view(true, "Initial Configuration");

    const arr qHome = C.getJointState();
    const arr qF = {0.20, -0.57, 1.29, -1, 0, 0, 0};
    const arr q_obs = {-0.19, -0.4, 0.95, 1, 0, 0, 0}; // Change this preset position with a heuristic
    arr path;
    bool is_aval_l = true;
    bool is_aval_r = true;
    bool is_feas = true;
    bool is_path_aval = false;
    bool is_path_blocked = false;
    int idx = 0;
    int obstacle_count = 0;
    int total_obstacle_count = 1;
    double filter = 0.01; // A percentage value
    double threshold = 3.7; // Cost threshold
    bool is_tool_aval = true;
    std::string tool = "";
    std::string target = "box";
    std::string contact_point = "";
    std::vector<std::string> tool_list = {"stick"};
    std::vector<std::string> gripper_list = {"l_l_gripper", "r_l_gripper"};

    if(!tool_list.empty())
    addMarker(C, C.getFrame("stick")->getPosition(), "stick_home", "world", 0.01, false);
    addMarker(C, C.getFrame("l_l_gripper")->getPosition(), "l_gripper_home", "world", 0.01, false);
    addMarker(C, C.getFrame("r_l_gripper")->getPosition(), "r_gripper_home", "world", 0.01, false);
    double calib = calibSkeleton(C);


    // Continue to find a path until a feasible path is found by removing obstacles
    do { // Convert this to for the combination of the obstacles
        cout << "Checking if path is available" << endl;
        is_path_aval = RRT(C2, qF, path);
        if(!is_path_aval) {
            is_path_blocked = true;
            cout << "Checking for obstacle no: " << obstacle_count << endl;
            if(obstacle_count != 0) {
                C2.getFrame(("obstacle_" + std::to_string(obstacle_count-1)).c_str())->setContact(1);
            }
            C2.getFrame(("obstacle_" + std::to_string(obstacle_count)).c_str())->setContact(0);
            obstacle_count++;
        }
        
    } while (!is_path_aval && (obstacle_count <= total_obstacle_count));
    obstacle_count--;

    if(!is_path_aval) {
        std::cerr << "No feasible path is available" << std::endl;
        return 0;
    }

   
    // Move the obstacle to a feasible position
    if(is_path_blocked) {
        cout << "Obstacle count: " << obstacle_count << endl;
        std::string obstacle = "obstacle_" + std::to_string(obstacle_count);
        addMarker(C, {q_obs(0), q_obs(1), q_obs(2)}, "waypoint", "world", 0.1,  false, {q_obs(3), q_obs(4), q_obs(5), q_obs(6)});
        
        std::string gripper = generateContactPoint(C, is_aval_l, is_aval_r, qHome, obstacle, "waypoint", filter, calib, contact_point, tool);
        C.view(true, "Contact Point Generated");

        is_feas = moveSkeleton(C, gripper, obstacle, tool, contact_point, "waypoint", is_aval_l, is_aval_r, threshold, calib)->eq - calib<= 3.7;
        cout << "Is move feas: " << is_feas << endl;
        if (!is_feas) {
            tool = useTool(C, path[idx], tool_list, gripper_list, "waypoint", obstacle, is_aval_l, is_aval_r);
        }
    }


    // Now move the robot along the path
    addMarker(C, candidateContactPoint(C, path, idx, false), "waypoint", "world", 0.01, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
    std::string gripper = generateContactPoint(C, is_aval_l, is_aval_r, qHome, target, "waypoint", filter, calib, contact_point, tool);
    C.view(true, "Contact Point Box");
    while(gripper == "" && is_tool_aval) {
        //C.viewer()->savePng(("video/config"+ std::to_string(img_count)).c_str()); 
        //img_count++;
        cout << "Object is not reachable" << endl;
        tool = useTool(C, path[idx], tool_list, gripper_list, "waypoint", target, is_aval_l, is_aval_r);
        C.view(false, "Tool use completed");
        gripper = generateContactPoint(C, is_aval_l, is_aval_r, qHome, target, "waypoint", filter, calib, contact_point, tool);
        C.view(true, "Contact Point Generated");
        idx++;
    }
    
    int isGripped = false;
    for(uint idx; idx < path.d0; idx++) {
        //C.viewer()->savePng(("video/config"+ std::to_string(img_count)).c_str()); 
        //img_count++; 
        cout << "Next waypoint: " << path(idx, 0) << " " << path(idx, 1) << " " << path(idx, 2) << endl;
        addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, "waypoint", "world", 0.01, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
        is_feas = moveSkeleton(C, gripper, target, tool, contact_point, "waypoint", is_aval_l, is_aval_r, threshold, calib)->eq -calib <= 3.7;
        cout << "Is feasible: " << is_feas << endl;
        if(!is_feas && !isGripped) {
            if(is_tool_aval) {
                tool = useTool(C, path[idx], tool_list, gripper_list, "waypoint", target, is_aval_l, is_aval_r);
            } else {
                homeSkeleton(C, "l_l_gripper", "r_l_gripper", "l_gripper_home", "r_gripper_home");
                calib = calibSkeleton(C);
            }
            gripper = generateContactPoint(C, is_aval_l, is_aval_r, qHome, target, "waypoint", filter, calib, contact_point, tool);
            C.view(true, "Contact Point Generated"); 

        } else if(!is_feas && isGripped) {
            gripper = generateContactPoint(C, is_aval_l, is_aval_r, qHome, target, "waypoint", filter, calib, contact_point, tool);
            C.view(true, "Contact Point Generated");
            isGripped = false;
        } else if (is_feas) {
            isGripped = true;
        }
    }
    cout << "The Target Is Reached"<< endl;
    
    return 0;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------FUNCTIONS------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat) {
    rai::Frame* marker = C.addFrame(name.c_str(), parent.c_str());
    marker->setShape(rai::ST_marker, {size, size, size, size});
    if(is_relative) {
        marker->setRelativePosition(pos);
    } else {
        marker->setPosition(pos);
    }
    marker->setQuaternion(quat); // w, x, y, z
    return *marker;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
bool RRT(rai::Configuration& C2, const arr& qF, arr& path, bool view, double rrt_extend_length) {
    arr q0 = C2.getJointState();
    std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
    auto problem = std::make_shared<ConfigurationProblem>(C2, true, 1e-3, 0);
    auto collisionPairs = rai::getParameter<StringA>("collisionPairs", {});
    auto coll = C2.getCollidablePairs();
    StringA collisionPairsString;
    for (const auto& frame : coll) {
        collisionPairsString.append(frame->name);
    }
    std::cout << coll.N <<  std::endl;
    if(collisionPairs.N) {
        problem->setExplicitCollisionPairs(collisionPairsString);
    }
    
    try {
        RRT_PathFinder rrt(*problem, q0, qF);

        bool isFeas = false;
        auto ret = std::make_shared<SolverReturn>();
        for (int i = 0; i < 10 && !isFeas; ++i) {
            //rrt = std::make_shared<RRT_PathFinder>(*problem, q0, qF, rrt_extend_length);
            RRT_PathFinder rrt2(*problem, q0, qF);
            
            // BU KISIM EKSTRA - RRT_PathFinder.cpp içinde PathFinder::solve kodundan
            ret->time -= rai::cpuTime();
            arr temp_path = rrt2.run();
            ret->time += rai::cpuTime();
            // BU KISIM EKSTRA - RRT_PathFinder.cpp içinde PathFinder::solve kodundan

            // BU KISIM GEREKLİ - RRT_PathFinder.cpp içinde PathFinder::solve kodundan
            ret->feasible = temp_path.N;
            ret->x = temp_path;
            ret->evals = rrt2.iters;

            isFeas = ret->feasible;
            path = ret->x;
        }
    } catch(...){
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    
    std::cout << "Path Prev: " << path.N << std::endl;
    if(path.N == 0) {
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    KOMO komo2(C2, path.d0, 1, 2, true);
    komo2.addControlObjective({}, 2, 1e0);
    komo2.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});
    komo2.initWithPath_qOrg(path);

    NLP_Solver solver(komo2.nlp(), 0);
    solver.setOptions(rai::OptOptions().set_stopTolerance(1e-2));
    solver.solve();

    path = komo2.getPath_qOrg();
    std::cout << "Path Post: " << path.N << std::endl;

    // Interpolate the path by increasing the size by twice
    arr newPath;
    for (uint i = 0; i < path.d0 - 1; ++i) {
        newPath.append(path[i]);
        newPath.append((path[i] + path[i + 1]) / 2);
    }

    newPath.resize(newPath.d0/7, 7);


    for (uint i = 0; i < newPath.d0 && view; ++i) {
        C2.setJointState(newPath[i]);  // Use path[i] to access the i-th configuration in the path
        C2.view();  // Use 'watch' instead of 'view' for consistency
        rai::wait(0.1);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    std::cout << "Path: " << newPath << std::endl;
    path = newPath;
    return true;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr getCameraView(rai::Configuration& C, const std::string& cam_name, const std::string& target, const double filter){
    static uint c;
    bool is_empty = false;
    int cam_frame_count = 0;
    arr pts_filtered;
    
    
    do {
        rai::CameraView V(C, true);
        OpenGL imgGl;
        byteA img;
        floatA depth;
        byteA segmentation;
        arr pts;
        rai::CameraView::Sensor sensor = V.addSensor("cam", ("cam_frame_" + std::to_string(cam_frame_count)).c_str(), 640, 360, 1.3, -1, {0.3, 5});

        pts_filtered.clear();
        // Get image and depth data from camera
        V.computeImageAndDepth(img, depth);
        segmentation = V.computeSegmentationImage();

        uint target_frame_id = C.getFrameIDs({target.c_str()})(0);
        uintA seg_id = V.computeSegmentationID();

        //imgGl.text="image";  imgGl.watchImage(img, true);   
        //imgGl.text="depth";  imgGl.watchImage(depth, true);
        imgGl.text="segmentation";  imgGl.watchImage(segmentation, true);
        
        // Mask the image and depth data
        byteA masked_img = img;
        for (int i = 0; i < masked_img.d0; i++) {
            for (int j = 0; j < masked_img.d1; j++) {
                if (seg_id(i, j) != target_frame_id) {
                    masked_img(i, j, 0) = 0;
                    masked_img(i, j, 1) = 0;
                    masked_img(i, j, 2) = 0;
                }
            } 
        }
        imgGl.watchImage(masked_img, true);
        
        // Mask the depth data by removing the masked pixels
        floatA depth_masked = depth;
        for (int i = 0; i < depth_masked.d0; i++) {
            for (int j = 0; j < depth_masked.d1; j++) {
                if (masked_img(i, j, 0) == 0 && masked_img(i, j, 1) == 0 && masked_img(i, j, 2) == 0) {
                    depth_masked(i, j) = -1;
                }
            }
        }
        //floatA depth_masked_new = convertPoint(C, depth_masked, "cam_frame_" + std::to_string(cam_frame_count));
        // Get the point cloud from the depth data and transform the points to the camera frame 
        depthData2pointCloud(pts, depth_masked, sensor.getFxycxy());

        std::string frameName = "points_" + std::to_string(c);
        rai::Frame* pts_frame = C.addFrame(frameName.c_str(), ("cam_frame_" + std::to_string(cam_frame_count)).c_str());


        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        int count = 1;
        for (int i = 0; i < pts.d0; i++) {
            for (int j = 0; j < pts.d1; j++) {
                if (!(pts(i, j, 0) == 0 && pts(i, j, 1) == 0 && pts(i, j, 2) == 0)) {
                    if (dis(gen) <= filter) {
                        pts_filtered.append({pts(i,j,0), pts(i,j,1), pts(i,j,2)});
                    } 
                    count ++;
                }
            }
        }

        cout << "Filtered Points Length: " << pts_filtered.d0 << endl;

        if(pts_filtered.d0 == 0) {
            if(is_empty){
                return arr();
            }
            homeSkeleton(C, "l_l_gripper", "r_l_gripper", "l_gripper_home", "r_gripper_home");
            cout << "Homing for camera" << endl;
            is_empty = true;
        } else {
            pts_filtered.reshape(pts_filtered.d0/3,3);
            if(pts_filtered.d0 <= 10){
                if(is_empty){
                    return arr();
                }
                homeSkeleton(C, "l_l_gripper", "r_l_gripper", "l_gripper_home", "r_gripper_home");
                cout << "Homing for camera" << endl;
                is_empty = true;
            } else {
                pts_frame->setPointCloud(pts_filtered);
                C.view(true, "Point cloud");
                is_empty = false;
            }

        }
    } while(is_empty);


    c++;
    cout << pts_filtered.d0 << " " << pts_filtered.d1 << endl;
    return pts_filtered;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr candidateContactPoint(rai::Configuration& C, const arr& pts, const int iter, bool isTransform){
    if(pts.d0 == 0) {
        std::cout << "No points found in the camera view" << std::endl;
        return {};
    }
    arr target_point;
    if(isTransform) {
        arr orig_rot = C.getFrame("world")->getRotationMatrix();
        arr orig_pos = C.getFrame("world")->getPosition();
        arr orig_point = {pts(iter, 0), pts(iter,1), pts(iter, 2)};
        arr target_rot = C.getFrame("cam_frame_0")->getRotationMatrix();
        arr target_pos = C.getFrame("cam_frame_0")->getPosition();

        orig_rot = inverse(orig_rot);
        arr hom_rot = target_rot * orig_rot;
        arr hom_pos = target_pos - hom_rot * orig_pos;
        target_point = hom_rot * orig_point + hom_pos;
    } else {
        target_point = {pts(iter, 0), pts(iter,1), pts(iter, 2)};
    }

    return target_point;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
const std::string generateContactPoint(rai::Configuration& C,  bool is_aval_l, bool is_aval_r, const arr qHome, const std::string& target, const std::string& waypoint, const double filter, const double calib, std::string& contact_point, std::string& tool){
    // Get the camera point cloud for the object
    static long int f1 = 0;
    static long int f2 = 0;
    arr pts = getCameraView(C, "cam", target, filter);
    arr sos_l_arr;
    arr sos_r_arr;
    int iter = 0;
    int max_iter = pts.d0;
    arr contactPoints;
    
    do {
        cout << "Iteration: " << iter << " / " << max_iter << endl;
        // Get the candidate contact point
        arr c_pts = candidateContactPoint(C, pts, iter);

        if(c_pts.N == 0) {
            C.setJointState(qHome);
            c_pts = candidateContactPoint(C, pts, iter);
            if(c_pts.N == 0) {
                cout << "No candidate contact point found" << endl;
                return "";
            }
        }

        // Add the candidate contact point to the configuration
        addMarker(C, c_pts, "candidate_point_" + std::to_string(f1) , target, 0.01, false);

        std::shared_ptr<SolverReturn> ret_l = moveSkeleton(C, "l_l_gripper", target, tool, "candidate_point_" + std::to_string(f1), waypoint.c_str(), is_aval_l, is_aval_r, 0, calib, true);
        std::shared_ptr<SolverReturn> ret_r = moveSkeleton(C, "r_l_gripper", target, tool, "candidate_point_" + std::to_string(f1), waypoint.c_str(), is_aval_l, is_aval_r, 0, calib, true);

        sos_l_arr.append(ret_l->eq - calib);
        sos_r_arr.append(ret_r->eq - calib);
        
    
        iter++;
        f1 ++;
    } while( iter < max_iter );

    sos_l_arr.append(INT_MAX);
    sos_r_arr.append(INT_MAX);
    double pt_idx;

    if(min(sos_l_arr) < min(sos_r_arr)) {
        cout << "The left gripper is closer to the object: " << min(sos_l_arr) << endl;
        pt_idx = argmin(sos_l_arr);
        contact_point =  "contact_point_" + std::to_string(f2);
        addMarker(C, candidateContactPoint(C, pts, pt_idx), contact_point, target, 0.2, false);
        C.view(true, "contacttttt");
        f2++;
        return "l_l_gripper";
        

    } else if(min(sos_r_arr) < min(sos_l_arr)) {
        cout << "The right gripper is closer to the object: " << min(sos_r_arr) << endl;
        pt_idx = argmin(sos_r_arr);
        contact_point =  "contact_point_" + std::to_string(f2);
        addMarker(C, candidateContactPoint(C, pts, pt_idx), contact_point, target, 0.2, false);
        C.view(true, "contacttttt");
        f2++;
        return "r_l_gripper";
    } else {
        cout << "No gripper is closer to the object" << endl;
        return "";
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
double calibSkeleton(rai::Configuration& C) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;

    S.addEntry({1, -1}, rai::SY_positionEq, {"l_l_gripper", "l_gripper_home"});
    S.addEntry({1, -1}, rai::SY_positionEq, {"r_l_gripper",  "r_gripper_home"});
    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();
    return ret->eq;
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::shared_ptr<SolverReturn> homeSkeleton(rai::Configuration& C, const std::string& l_gripper, const std::string& r_gripper, const std::string& l_gripper_home, const std::string& r_gripper_home) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;

    S.addEntry({1, -1}, rai::SY_positionEq, {l_gripper.c_str(), l_gripper_home.c_str()});
    S.addEntry({1, -1}, rai::SY_positionEq, {r_gripper.c_str(), r_gripper_home.c_str()});
    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    komo_path->pathConfig.viewer()->raiseWindow();
    komo_path->view_play(false, 0.2, ("video/config"+ std::to_string(img_count)).c_str());
    img_count++;
    arr state = komo_path->pathConfig.getFrameState();
    double frame_count = komo_path->pathConfig.getFrameNames().d1;
    state = state.rows(state.d0-frame_count,state.d0);
    C.setFrameState(state);
    C.view(false);
        
    return ret;
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::shared_ptr<SolverReturn> moveSkeleton(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& tool, const std::string& contact_point, const std::string& waypoint, bool& is_aval_l, bool& is_aval_r, const double threshold, const double calib, const bool isTrial) {
    
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;    
    std::shared_ptr<KOMO> komo_path;
    if((gripper == "l_l_gripper" && !is_aval_l) || (gripper == "r_l_gripper" && !is_aval_r)){
        S.addEntry({1, -1}, rai::SY_stable, {gripper.c_str(), tool.c_str()});
        S.addEntry({1, -1}, rai::SY_touch, {gripper.c_str(), tool.c_str()});
        S.addEntry({1.2, -1}, rai::SY_poseEq, {tool.c_str(), (tool + "_home").c_str()});
        S.addEntry({1.5, -1}, rai::SY_stable, {gripper.c_str(), target.c_str()});
        S.addEntry({1.5, -1}, rai::SY_touch, {gripper.c_str(), target.c_str()});
        S.addEntry({1.8, -1}, rai::SY_poseEq, {target.c_str(), waypoint.c_str()});
        komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
        
        komo_path->addObjective({1.5, -1}, FS_positionDiff, {gripper.c_str(), contact_point.c_str()}, OT_sos, {1e2});        
    } else {
        S.addEntry({1, -1}, rai::SY_stable, {gripper.c_str(), target.c_str()});
        S.addEntry({1, -1}, rai::SY_touch, {gripper.c_str(), target.c_str()});
        S.addEntry({1.2, -1}, rai::SY_poseEq, {target.c_str(), waypoint.c_str()});
        komo_path = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
        
        komo_path->addObjective({1, -1}, FS_positionDiff, {gripper.c_str(), contact_point.c_str()}, OT_sos, {1e2});
    }

    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    cout << "Move Cost: " <<ret->eq - calib<< endl;

    if(!isTrial){
        cout <<komo_path->report(true, true, true) <<endl;

        if(((ret->eq - calib) <= threshold) && !isTrial) {
            komo_path->pathConfig.viewer()->raiseWindow();
            komo_path->view_play(false, 0.2, ("video/config"+ std::to_string(img_count)).c_str());
            img_count++;
            arr state = komo_path->pathConfig.getFrameState();
            double frame_count = komo_path->pathConfig.getFrameNames().d1;
            state = state.rows(state.d0-frame_count,state.d0);
            C.setFrameState(state);
            C.view(false);
            is_aval_l = true;
            is_aval_r = true;
        } 
    }
    return ret;
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
double toolSkeleton(rai::Configuration& C, const std::string& tool, const std::string& target, const std::string& gripper, const std::string& final_pose, const bool isTrial) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;
    S.addEntry({1, -1}, rai::SY_stable, {gripper.c_str(), tool.c_str()});
    S.addEntry({1, -1}, rai::SY_touch, {gripper.c_str(), tool.c_str()});
    S.addEntry({1.2, -1}, rai::SY_stable, {tool.c_str(), target.c_str()});
    S.addEntry({1.2, -1}, rai::SY_touch, {tool.c_str(), target.c_str()});
    S.addEntry({1.6, 1.6}, rai::SY_poseEq, {final_pose.c_str(), target.c_str()});
  
    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 16, 1e0, 1e-2, 1e-3, 1e1);
    komo_path->addObjective({1.4, -1}, FS_quaternionDiff, {tool.c_str(), target.c_str()}, OT_sos, {1e1});  
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    cout << "Tool Cost: " << ret->eq << endl;

    if(!isTrial) {
        komo_path->pathConfig.viewer()->raiseWindow();
        komo_path->view_play(false, 0.2, ("video/config"+ std::to_string(img_count)).c_str());
        img_count++;
        arr state = komo_path->pathConfig.getFrameState();
        double frame_count = komo_path->pathConfig.getFrameNames().d1;
        state = state.rows(state.d0-frame_count,state.d0);
        C.setFrameState(state);
    }

    return ret->eq;
} 

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void toolSelection(rai::Configuration& C, const std::vector<std::string>& tool_list, const std::vector<std::string>&  gripper_list, const std::string waypoint, const std::string target, bool& is_aval_l, bool& is_aval_r, std::string& gripper_out, std::string& tool_out){
    int s1 = tool_list.size();
    int s2 = gripper_list.size();
    std::vector<std::vector<double>> cost_arr(s1, std::vector<double>(s2, 0.0));
    for (int i = 0; i < s1; i++) {
        std::string tool = tool_list[i];
        for  (int j = 0; j < s2; j++){
            std::string gripper = gripper_list[j];
            cout << "Tool: " << tool << " Gripper: " << gripper << endl;
            double cost = toolSkeleton(C, tool, target, gripper, waypoint, true);
            cost_arr[i][j] = cost;
        }
    }   

    double min_value = std::numeric_limits<double>::max(); // Set to a large value initially
    int min_row = -1;
    int min_col = -1;
    for (int i = 0; i < cost_arr.size(); ++i) {
        for (int j = 0; j < cost_arr[i].size(); ++j) {
            if (cost_arr[i][j] < min_value) {
                min_value = cost_arr[i][j];
                min_row = i;
                min_col = j;
            }
        }
    }
    tool_out = tool_list[min_row];
    gripper_out = gripper_list[min_col];

    cout << "Best Tool: " << tool_out << endl;
    cout << "Best Gripper: " << gripper_out << endl;
    if(gripper_out == "r_l_gripper") {
        is_aval_r = false;
        is_aval_l = true;
    }
    else {
        is_aval_l = false;
        is_aval_r = true;
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::string useTool(rai::Configuration& C, const arr path_point, const std::vector<std::string>& tool_list, const std::vector<std::string>&  gripper_list, const std::string waypoint, const std::string target, bool& is_aval_l, bool& is_aval_r){
    cout << "Tool use initiated!" << endl;
    std::string gripper_tool;
    std::string tool; 
    addMarker(C, {path_point(0), path_point(1), path_point(2)}, waypoint, "world", 0.01, false,  {path_point(3), path_point(4), path_point(5), path_point(6)});
    toolSelection(C, tool_list, gripper_list, waypoint, target, is_aval_l, is_aval_r, gripper_tool, tool);
    toolSkeleton(C, tool, target, gripper_tool, waypoint);
    return tool;
}




