#include "HMAPBiman.h"

HMAPBiman::HMAPBiman(rai::Configuration C, rai::Configuration C2, arr qF, arr q_obs, std::string target, int total_obstacle_count, std::vector<std::string> tool_list, std::vector<std::string> gripper_list, double filter, std::string video_path, int waypoint_factor, bool view){
    this->C = C;
    this->C2 = C2;
    this->qHome = this->C.getJointState();
    this->is_aval_l = true;
    this->is_aval_r = true;
    this->is_feas = true;
    this->is_path_aval = false;
    this->is_path_blocked = false;
    this->idx = 0;
    this->obstacle_count = 0;
    this->total_obstacle_count = total_obstacle_count;
    this->filter = filter;
    this->tool = "";
    this->target = target;
    this->contact_point = "";
    this->tool_list = tool_list;
    this->gripper_list = gripper_list;
    this->calib = 0;
    this->threshold = 1;
    this->qF = qF;
    this->q_obs = q_obs;
    this->path = {};
    this->img_count = 0;
    this->contact_points = {};
    this->view = view;
    this->video_path = video_path;
    this->fail_limit = 10;
    this->waypoint_factor = waypoint_factor;
    this->state_all = {};
    this->state_count = 0;
    if(tool_list.empty()) {
        this->is_tool_aval = false;
    } else {
        this->is_tool_aval = true;
    }
}

bool HMAPBiman::run(){
    rai::Configuration C_base;
    C_base = C;

    if(view) C.view(true, "Initial Configuration");
    if(!tool_list.empty())
    addMarker(C, C.getFrame("stick")->getPosition(), "stick_home", "world", 0.01, false);

    addMarker(C, C.getFrame("l_l_gripper")->getPosition(), "l_gripper_home", "world", 0.01, false);
    addMarker(C, C.getFrame("r_l_gripper")->getPosition(), "r_gripper_home", "world", 0.01, false);

    calib = calibSkeleton(C);
    cout << "Calibration: " << calib << endl;
    cout << "Threshold: " << threshold << endl;

    // Continue to find a path until a feasible path is found by removing obstacles
    do { // Convert this to for the combination of the obstacles
        cout << "Checking if path is available" << endl;
        is_path_aval = RRT(C2, path, view);
        if(!is_path_aval && total_obstacle_count != 0 && (obstacle_count < total_obstacle_count)) {
            is_path_blocked = true;
            cout << "Checking for obstacle no: " << obstacle_count << endl;
            if(obstacle_count != 0) {
                C2.getFrame(("obstacle_" + std::to_string(obstacle_count-1)).c_str())->setContact(1);
            }
            C2.getFrame(("obstacle_" + std::to_string(obstacle_count)).c_str())->setContact(0);
            obstacle_count++;
        }
        
    } while (!is_path_aval && (obstacle_count <= total_obstacle_count) && total_obstacle_count != 0);

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
        std::string gripper = generateContactPoint(C, obstacle, "waypoint", contact_point);
        is_feas = moveSkeleton(C, gripper, obstacle, contact_point, "waypoint")->eq - calib <= threshold;
        cout << "Is move feas: " << is_feas << endl;
        if (!is_feas && is_tool_aval) {
            tool = useTool(C, path[idx], "waypoint", obstacle);
        }
    }

    std::vector<std::string> gripper_list;
    std::vector<std::string> target_list;
    std::vector<std::string> waypoint_list;
    std::vector<std::string> contact_point_list;
    std::string waypoint = "waypoint_" + std::to_string(idx);
    std::string cp = "contact_point_" + std::to_string(idx);

    // Check if there is enough clearance to reach the object
    addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
    addMarker(C_base, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});

    std::string gripper = generateContactPoint(C, target, waypoint, contact_point);
    addMarker(C_base, C.getFrame("contact_point")->getPosition(), cp, target, 0.1, false);
    while(gripper == "" && is_tool_aval) {
        cout << "Object is not reachable" << endl;
        tool = useTool(C, path[idx], waypoint, target);
        gripper = generateContactPoint(C, target, waypoint, contact_point);
        contact_points.append(C.getFrame("contact_point")->getPosition());
        idx++;
    }

    // Now move the robot along the path
    while(idx < path.d0){
        waypoint = "waypoint_" + std::to_string(idx);
        //addMarker(C, C.getFrame("contact_point")->getPosition(), cp, target, 0.1, false);
        // If the ideal gripper has tool at hand make it place it
        if(gripper == "l_l_gripper" && !is_aval_l) {
            homeTool(C, gripper, tool);
        } else if((gripper == "r_l_gripper" && !is_aval_r)) {
            homeTool(C, gripper, tool);
        }

        cout << "Next waypoint: " << path(idx, 0) << " " << path(idx, 1) << " " << path(idx, 2) << endl;
        addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.001, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
        addMarker(C_base, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.001, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});

        if(view)
        C.view(false, "Waypoint");
        
        if(gripper != ""){
            is_feas = moveSkeleton(C, gripper, target, contact_point, waypoint)->eq - calib <= threshold;
        } else {
            is_feas = false;
        }
        
        cout << "Is feasible: " << is_feas << endl;
        
        // If the path is not feasible, use the tool to reach the object, if the tool is not available, calibrate the robot as a workaround 
        if(!is_feas) {
            
            if(is_tool_aval) {
                tool = useTool(C, path[idx], waypoint, target);
            } else {
                homeSkeleton(C, "l_l_gripper", "r_l_gripper", "l_gripper_home", "r_gripper_home");
                calib = calibSkeleton(C);
            }
            gripper = generateContactPoint(C, target, waypoint, contact_point);
            cp = "contact_point_" + std::to_string(idx);
            addMarker(C_base, C.getFrame("contact_point")->getPosition(), cp, target, 0.1, false);
        } else {
            gripper_list.push_back(gripper);
            target_list.push_back(target);
            waypoint_list.push_back(waypoint);
            contact_point_list.push_back(cp);
            idx ++;
        }
        if(gripper != "") {
            contact_points.append(C.getFrame(contact_point.c_str())->getPosition());
        }
    }
    cout << "The HMAP is completed."<< endl;
    completeSkeleton(C_base, gripper_list, target_list, waypoint_list, contact_point_list, idx);
    return 1;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::completeSkeleton(rai::Configuration& C, std::vector<std::string> gripper_list, std::vector<std::string> target_list, std::vector<std::string> waypoint_list, std::vector<std::string> contact_point_list, int count) {
    C.view(true, "Last");
    cout << "Optimizing as a whole" << endl;
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;    
    double multiplier = 0.2;
    double s1 = 0.7;
    double s2 = 1;
    double s3 = 1.3;
    double s4 = 1.6;

    std::shared_ptr<KOMO> komo_path;
    std::string gripper = "";
    std::string target = "";
    std::string waypoint = "";

    //S.addExplicitCollisions({"box", "tunnel_top", "tunnel_side", "tunnel_side2", "tunnel_top_2", "tunnel_side_2", "tunnel_side2_2"});
    
    double limit = 0;
    int foo = 0;
    arr limit_list = ones(gripper_list.size()) * -1;
    arr cp_st = {};
    std::string contact_foo = "";

    for(uint i = 0; i < gripper_list.size(); i++){
        cout << "Limit: " << limit << endl;
        if(i == 0){
            contact_foo = contact_point_list[i];
            limit = s3 + multiplier;
        }
        else {
            if(contact_foo == contact_point_list[i]){
                limit += multiplier;
            } else {
                contact_foo = contact_point_list[i];
                cout << "i: " << i << endl;
                limit += multiplier;

                for(uint j = foo; j < i; j++){
                    limit_list(j) = limit;
                }
                limit += s3;
                foo = i;
            }
        }
    }

    cout << "Limit List: " << limit_list << endl;
    std::string contact_point = "";
    std::string contact_point_prev = "";
    double D = 0;

    for(uint i = 0; i < count; i++){
        gripper = gripper_list[i];
        target  = target_list[i];
        waypoint = waypoint_list[i];
        contact_point = contact_point_list[i];
        if(contact_point != contact_point_prev){
            D += multiplier;
            //if(contact_point_prev != "") 
            S.addEntry({s1+D, limit_list(i)}, rai::SY_touch , {gripper.c_str(), contact_point.c_str()});
            cout << "D: " << D+s1 << " Limit: " << limit_list(i) << " State: Change, Stable" << " Gripper: " << gripper << " Contact: " << contact_point << " Waypoint: " << waypoint << endl;
            S.addEntry({s2+D, limit_list(i)}, rai::SY_stable, {gripper.c_str(), target.c_str()});
            cout << "D: " << D +s2 << " Limit: " << limit_list(i) << " State: Change, Touch" << " Gripper: " << gripper << " Contact: " << contact_point << " Waypoint: " << waypoint << endl;
            S.addEntry({s3+D, s3+D}, rai::SY_poseEq, {target.c_str(), waypoint.c_str()});
            cout << "D: " << D +s3<< " Limit: " << limit_list(i) << " State: Change, PoseEq" << " Gripper: " << gripper << " Contact: " << contact_point << " Waypoint: " << waypoint << endl;
            D += s3;
            cp_st.append(D);
        } else {
            D += multiplier;
            S.addEntry({D, D}, rai::SY_poseEq, {target.c_str(), waypoint.c_str()});
            cp_st.append(D);
            cout << "D: " << D << " Limit: " << limit_list(i) << " State: Move" << " Gripper: " << gripper << " Contact: " << contact_point << " Waypoint: " << waypoint << endl;
        }
        contact_point_prev = contact_point;
    }
    cout << "CP list: " << cp_st << endl;
    D = 0;
    komo_path = S.getKomo_path(C, 2*count, 1e0, 1e-2, 1e-3, 1e2);
    contact_point_prev = "";
    for(uint i = 0; i < count; i++){
        target = gripper_list[i];
        gripper = gripper_list[i];
        contact_point = contact_point_list[i];
        if(contact_point_prev != contact_point){
            if(contact_point_prev == ""){
                komo_path->addModeSwitch({cp_st(i), limit_list(i)}, rai::SY_quasiStaticOn, {"table", "box"}, true);
            } else {
                komo_path->addModeSwitch({cp_st(i), limit_list(i)}, rai::SY_quasiStaticOn, {"table", "box"}, false);
            }
            komo_path->addObjective({cp_st(i), limit_list(i)}, FS_positionDiff, {gripper.c_str(), contact_point.c_str()}, OT_sos, {1e2});
            cout << "D: " << cp_st(i) << " Limit: " << limit_list(i) << " State: CP" << " Gripper: " << gripper << " Contact: " << contact_point  << endl;
        }
        contact_point_prev = contact_point;
    }
    ////komo_path->addObjective({}, FS_accumulatedCollisions, {}, OT_sos, {1e2});

    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();
    cout << "Move Cost: " <<ret->eq<< endl;
    C.view(true, "Optimized");
    cout <<komo_path->report(true, true, true) <<endl;
    komo_path->pathConfig.viewer()->raiseWindow();
    komo_path->view_play(true, 0.2);
    arr state = komo_path->pathConfig.getFrameState();
    double frame_count = komo_path->pathConfig.getFrameNames().d1;
    cout << "State Count: " << state.d0 << " " << state.d1 << endl;
    state = state.rows(state.d0-frame_count,state.d0);
    C.setFrameState(state);
    C.view(true, "Move Skeleton");

}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::getContactPoints(){
    return contact_points;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::getPath(){
    return path;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
rai::Frame& HMAPBiman::addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat) {
    rai::Frame* marker = C.addFrame(name.c_str(), parent.c_str());
    marker->setShape(rai::ST_marker, {size, size, size, size});
    if(is_relative) {
        marker->setRelativePosition(pos);
    } else {
        marker->setPosition(pos);
    }
    if(quat.N == 0) {
        quat = C.getFrame(target.c_str())->getQuaternion();
    }
    marker->setQuaternion(quat); // w, x, y, z
    return *marker;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
bool HMAPBiman::RRT(rai::Configuration& C2, arr& path, bool view) {
    arr q0 =  C2.getJointState();
    std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
    addMarker(C2, {q0(0), q0(1), q0(2)}, "cp0", "world", 0.2, false, {q0(3), q0(4), q0(5), q0(6)});
    addMarker(C2, {qF(0), qF(1), qF(2)}, "cp", "world", 0.2, false, {qF(3), qF(4), qF(5), qF(6)});

    if(view)
    C2.view(false, "RRT");
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
        rrt.verbose = -1;
        bool isFeas = false;
        auto ret = std::make_shared<SolverReturn>();
        for (int i = 0; i < 30 && !isFeas; ++i) {
            cout << "RRT Iteration: " << i << endl;
            //rrt = std::make_shared<RRT_PathFinder>(*problem, q0, qF, rrt_extend_length);
            RRT_PathFinder rrt2(*problem, q0, qF);
            rrt2.verbose = -1;
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
        std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    
    std::cout << "Path Prev: " << path.N << std::endl;
    if(path.N == 0) {
        std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    KOMO komo2(C2, path.d0, 1, 2, true);
    komo2.addControlObjective({}, 2, 1e0);
    komo2.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});
    komo2.initWithPath_qOrg(path);

    NLP_Solver solver(komo2.nlp(), -1);
    solver.setOptions(rai::OptOptions().set_stopTolerance(1e-2));
    solver.solve();

    path = komo2.getPath_qOrg();

    arr newPath;
    int factor = waypoint_factor;  // Set the interpolation factor (number of intervals between points)
    for (uint i = 0; i < path.d0 - 1; ++i) {
        newPath.append(path[i]);
        for (int j = 1; j < factor; ++j) {
            newPath.append(path[i] + j * (path[i + 1] - path[i]) / factor);
        }
    }

    newPath.append(path[path.d0 - 1]);  // Append the last point
    newPath.resize(newPath.d0/path.d1, path.d1);
    for (uint i = 0; i < newPath.d0; ++i) {
        C2.setJointState(newPath[i]);  // Use path[i] to access the i-th configuration in the path
        if(view) {
            addMarker(C2, {newPath(i,0), newPath(i,1), newPath(i,2)}, "cp" + std::to_string(i), "world", 0.1, false, {newPath(i,3), newPath(i,4), newPath(i,5), newPath(i,6)});
            C2.view(false);
            rai::wait(0.1);
        }
    }

    path = newPath;
    return true;
}
 /*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::getCameraView(rai::Configuration& C, const std::string& cam_name, const std::string& target, const double filter){
    static uint c;
    bool is_empty = false;
    int cam_frame_count = 0;
    arr pts_filtered;

    do {
        //homeSkeleton(C, "l_l_gripper", "r_l_gripper", "l_gripper_home", "r_gripper_home");
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
        //imgGl.text="segmentation";  imgGl.watchImage(segmentation, true);
        
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
        //imgGl.watchImage(masked_img, true);
        
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
            if(pts_filtered.d0 <= 15){
                if(is_empty){
                    return arr();
                }
                homeSkeleton(C, "l_l_gripper", "r_l_gripper", "l_gripper_home", "r_gripper_home");
                cout << "Homing for camera" << endl;
                is_empty = true;
            } else {
                //pts_frame->setPointCloud(pts_filtered);
                //cout << pts_frame->getPosition() << endl;
                //addMarker(C, candidateContactPoint(C, pts_filtered, 2, true), "contact_pointads", "box", 0.1, false);
                //C.view(true, "Point cloud");
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
arr HMAPBiman::candidateContactPoint(rai::Configuration& C, const arr& pts, const int iter, bool isTransform){
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
const std::string HMAPBiman::generateContactPoint(rai::Configuration& C, const std::string& target, const std::string& waypoint, std::string& contact_point){
    // Get the camera point cloud for the object
    static long int f1 = 0;
    static long int f2 = 0;
    arr pts = getCameraView(C, "cam", target, filter);
    arr sos_l_arr;
    arr sos_r_arr;
    int iter = 0;
    int max_iter;
    arr contactPoints;
    int fail_count = 0;
    int heuristic = 0;
    if(heuristic == 0){
        max_iter = 40;
        for(iter = 0; iter < max_iter; iter++){

            cout << "Iteration: " << iter << " / " << max_iter << endl;
            if(pts.d0 == 0) return "";

            double pt_idx = rnd.num(0, pts.d0 - 1);

            arr c_pts = candidateContactPoint(C, pts, pt_idx, true);
            if(c_pts.N == 0) {
                cout << "No candidate contact point found" << endl;
                return "";   
            }

            addMarker(C, c_pts, "candidate_point" , target, 0.0001, false);

            std::shared_ptr<SolverReturn> ret_l = moveSkeleton(C, "l_l_gripper", target, "candidate_point", waypoint.c_str(), true);
            std::shared_ptr<SolverReturn> ret_r = moveSkeleton(C, "r_l_gripper", target, "candidate_point", waypoint.c_str(), true);

            double l_cost = ret_l->eq - calib;
            double r_cost = ret_r->eq - calib;

            if(l_cost < threshold && l_cost < r_cost) {
                cout << "The left gripper is closer to the object: " << l_cost << endl;
                contact_point =  "contact_point";
                addMarker(C, c_pts, contact_point, target, 0.1, false);
                C.view(false);
                return "l_l_gripper";
            } else if(r_cost < threshold && r_cost < l_cost) {
                cout << "The right gripper is closer to the object: " << r_cost << endl;
                contact_point =  "contact_point";
                addMarker(C, c_pts, contact_point, target, 0.1, false);
                C.view(false);
                return "r_l_gripper";
            } 

        }
        return "";

    } else {
        do {
            max_iter = pts.d0;
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
            addMarker(C, c_pts, "candidate_point" , target, 0.001, false);

            std::shared_ptr<SolverReturn> ret_l = moveSkeleton(C, "l_l_gripper", target, "candidate_point", waypoint.c_str(), true);
            std::shared_ptr<SolverReturn> ret_r = moveSkeleton(C, "r_l_gripper", target, "candidate_point", waypoint.c_str(), true);

            sos_l_arr.append(ret_l->eq - calib);
            sos_r_arr.append(ret_r->eq - calib);
            
            if((ret_l->eq - calib) > threshold && (ret_r->eq - calib) > threshold) {
                fail_count++;
            } else {
                fail_count = 0;
            }
        
            iter++;
            f1 ++;
        } while( iter < max_iter && fail_count < fail_limit);

        sos_l_arr.append(INT_MAX);
        sos_r_arr.append(INT_MAX);
        double pt_idx;

        if(min(sos_l_arr) < min(sos_r_arr)) {
            cout << "The left gripper is closer to the object: " << min(sos_l_arr) << endl;
            pt_idx = argmin(sos_l_arr);
            contact_point =  "contact_point";
            addMarker(C, candidateContactPoint(C, pts, pt_idx), contact_point, target, 0.1, false);
            f2++;
            return "l_l_gripper";
            
        } else if(min(sos_r_arr) < min(sos_l_arr)) {
            cout << "The right gripper is closer to the object: " << min(sos_r_arr) << endl;
            pt_idx = argmin(sos_r_arr);
            contact_point =  "contact_point";
            addMarker(C, candidateContactPoint(C, pts, pt_idx), contact_point, target, 0.1, false);
            f2++;
            return "r_l_gripper";
        } else {
            cout << "No gripper is closer to the object" << endl;
            return "";
        }
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
double HMAPBiman::calibSkeleton(rai::Configuration& C) {
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
std::shared_ptr<SolverReturn> HMAPBiman::homeSkeleton(rai::Configuration& C, const std::string& l_gripper, const std::string& r_gripper, const std::string& l_gripper_home, const std::string& r_gripper_home) {
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

    if(view){
        komo_path->pathConfig.viewer()->raiseWindow();
        komo_path->view_play(false, 0.2, (video_path+ std::to_string(img_count)).c_str());
    }

    img_count++;
    arr state = komo_path->pathConfig.getFrameState();

    state_all.push_back(komo_path);
    state_count ++;
    double frame_count = komo_path->pathConfig.getFrameNames().d1;
   
    state = state.rows(state.d0-frame_count,state.d0);
    C.setFrameState(state);

    return ret;
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::shared_ptr<SolverReturn> HMAPBiman::moveSkeleton(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& contact_point, const std::string& waypoint, const bool isTrial) {
    
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;    
    std::shared_ptr<KOMO> komo_path;

    
    S.addEntry({0.7, -1}, rai::SY_stable, {gripper.c_str(), target.c_str()});
    S.addEntry({1, -1}, rai::SY_touch, {gripper.c_str(), target.c_str()});
    S.addEntry({1.3, -1}, rai::SY_poseEq, {target.c_str(), waypoint.c_str()});
    komo_path = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e2);
    
    komo_path->addObjective({1.3, -1}, FS_positionDiff, {gripper.c_str(), contact_point.c_str()}, OT_sos, {1e1});
    //komo_path->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    cout << "Move Cost: " <<ret->eq - calib<< endl;

    if(!isTrial){
        if(view)
        cout <<komo_path->report(true, true, true) <<endl;
        if(((ret->eq - calib) <= threshold) && !isTrial) {
            if(view){
                komo_path->pathConfig.viewer()->raiseWindow();
                komo_path->view_play(false, 0.2, (video_path+ std::to_string(img_count)).c_str());
                img_count++;
            }

            arr state = komo_path->pathConfig.getFrameState();
            state_all.push_back(komo_path);
            state_count++;
            double frame_count = komo_path->pathConfig.getFrameNames().d1;
            cout << "State Count: " << state.d0 << " " << state.d1 << endl;
            state = state.rows(state.d0-frame_count,state.d0);
            C.setFrameState(state);
            if(view)
            C.view(false, "Move Skeleton");
            if(gripper == "r_l_gripper" && !is_aval_r) is_aval_r = true;
            else if(gripper == "l_l_gripper" && !is_aval_l) is_aval_l = true;
        } 
    }
    return ret;
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
double HMAPBiman::toolSkeleton(rai::Configuration& C, const std::string& tool, const std::string& target, const std::string& gripper, const std::string& final_pose, const bool isTrial) {
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
    komo_path->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    cout << "Tool Cost: " << ret->eq << endl;

    if(!isTrial) {
        if(view){
            komo_path->pathConfig.viewer()->raiseWindow();
            komo_path->view_play(false, 0.2, (video_path+ std::to_string(img_count)).c_str());
            img_count++;
        }
        arr state = komo_path->pathConfig.getFrameState();
        state_all.push_back(komo_path);
        state_count++;
        double frame_count = komo_path->pathConfig.getFrameNames().d1;
        state = state.rows(state.d0-frame_count,state.d0);
        C.setFrameState(state);
    }

    return ret->eq;
} 

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::toolSelection(rai::Configuration& C, const std::string waypoint, const std::string target, std::string& gripper_out, std::string& tool_out){
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
void HMAPBiman::homeTool(rai::Configuration& C, const std::string& gripper, const std::string& tool) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;

    S.addEntry({1, -1}, rai::SY_stable, {gripper.c_str(), tool.c_str()});
    S.addEntry({1, -1}, rai::SY_touch, {gripper.c_str(), tool.c_str()});
    S.addEntry({2, -1}, rai::SY_positionEq, {tool.c_str(), (tool + "_home").c_str()});
    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    if(view){
        komo_path->pathConfig.viewer()->raiseWindow();
        komo_path->view_play(false, 0.2, (video_path + std::to_string(img_count)).c_str());
        img_count++;
    }

    arr state = komo_path->pathConfig.getFrameState();
    state_all.push_back(komo_path);
    state_count++;
    double frame_count = komo_path->pathConfig.getFrameNames().d1;
    state = state.rows(state.d0-frame_count,state.d0);
    C.setFrameState(state);
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::string HMAPBiman::useTool(rai::Configuration& C, const arr path_point, const std::string waypoint, const std::string target){
    cout << "Tool use initiated!" << endl;
    std::string gripper_tool;
    std::string tool; 
    addMarker(C, {path_point(0), path_point(1), path_point(2)}, waypoint, "world", 0.01, false,  {path_point(3), path_point(4), path_point(5), path_point(6)});
    toolSelection(C, waypoint, target, gripper_tool, tool);
    toolSkeleton(C, tool, target, gripper_tool, waypoint);
    return tool;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::displaySolution(){
    std::shared_ptr<KOMO> komo_path;
    for(uint i = 0; i < state_count; i++){
        komo_path = state_all[i];
        komo_path->pathConfig.viewer("path", true)->raiseWindow();
        komo_path->view_play(false, 0.1);
        komo_path->view_close();
        cout << i << "/" << state_count << endl;
    }
    return;
}

