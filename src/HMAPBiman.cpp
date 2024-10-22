#include "HMAPBiman.h"

HMAPBiman::HMAPBiman(rai::Configuration C, rai::Configuration C2, arr qF, arr q_obs, std::string target, std::string interacted_target, int total_obstacle_count, std::vector<std::string> tool_list, std::vector<std::string> gripper_list, double filter, std::string video_path, int waypoint_factor, bool view){
    this->C = C;
    this->C2 = C2;
    this->C_copy = C;
    this->qHome = this->C.getJointState();
    this->is_save_C = false;
    this->is_path_given = false;
    this->is_model_aval = false;
    this->is_aval_list = {};
    this->offline_cp_model = {};
    this->is_feas = true;
    this->is_path_aval = false;
    this->is_path_blocked = false;
    this->idx = 0;
    this->obstacle_count = -1;
    this->total_obstacle_count = total_obstacle_count;
    this->filter = filter;
    this->tool = "";
    this->gripper_count = gripper_list.size();
    this->interacted_target = interacted_target; // This is the handle. If the object is a whole the target and interacted target will be the same
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
    this->states = {};
    this->C_views = {};
    this->state_count = 0;
    this->is_dynamic = false;
    this->dyn_obs_idx = -1;
    this->dyn_obs_pos = {};

    if(tool_list.empty()) {
        this->is_tool_aval = false;
    } else {
        this->is_tool_aval = true;
    }
    this->Cs = {};

    for(uint i = 0; i < gripper_count; i++){
        is_aval_list.push_back(true);
    }

}

bool HMAPBiman::run(){
    rai::Configuration C_base;
    C_base = C;

    if(view) C.view(true, "Initial Configuration");

    if(!tool_list.empty()){
        for (uint i = 0; i < tool_list.size(); i++){
            std::string t = tool_list[i];
            addMarker(C, C.getFrame(t.c_str())->getPosition(), (t + "_home"), "world", 0.01, false);
        }
    }
    
    for (uint i = 0; i < gripper_count; i++){
        std::string g = gripper_list[i];
        addMarker(C, C.getFrame(g.c_str())->getPosition(), (g + "_home"), "world", 0.01, false, C.getFrame(g.c_str())->getQuaternion());
        addMarker(C_base, C.getFrame(g.c_str())->getPosition(), (g + "_home"), "world", 0.01, false, C_base.getFrame(g.c_str())->getQuaternion());
    }
    
    calib = calibSkeleton(C);
    cout << "Calibration: " << calib << endl;
    cout << "Threshold: " << threshold << endl;

    // PATH PLANNING
    cout << "Generating Path Plan" << endl;
    bool is_path_feas = generatePathPlan(path, qF);
    if(!is_path_feas) return 0;

    std::vector<std::string> gripper_list;
    std::vector<std::string> target_list;
    std::vector<std::string> waypoint_list;
    std::vector<std::string> contact_point_list;
    std::string waypoint = "waypoint";

    // Check if there is enough clearance to reach the object
    addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
    addMarker(C_base, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});

    cout << "Checking if object is reachable" << endl;
    // CHECK IF TARGET IS VISIBLE 
    std::string gripper = generateContactPoint(C, target, interacted_target, waypoint);
    while(gripper == "" && is_tool_aval) {
        cout << "Object is not reachable" << endl;
        tool = useTool(C, waypoint, target);
        gripper = findContactPoint(C, target, interacted_target, waypoint);
        idx++;
        addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
        addMarker(C_base, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
    }

    int sol_fail_limit = 3;
    int sol_fail_count = 0;

    cout << "The main loop"<< endl;
    // THE MAIN LOOP
    // Now move the robot along the path
    while(idx < path.d0 ) {

        // ADD DYNAMIC OBSTACLE
        if(is_dynamic && idx == path.d0 / 2) {
            C.getFrame("obstacle")->setPosition({path(idx+5,0), path(idx+5,1), path(idx+5,2)});
            C2.getFrame("obstacle")->setPosition({path(idx+5,0), path(idx+5,1), path(idx+5,2)});

            C2.setJointState(C.getFrame(target.c_str())->getPose());
            arr n_path = {};

            if(RRT(C2, n_path, qF, view)){
                path = n_path;
                idx = 0;
                is_dynamic = false;
            }
        } 
        // CHANGE THE THRESHOLD (OPTIONAL)
        if(idx == 0) {
            threshold = 4;
        } else {
            threshold = 1;
        } 
       
        int frame_err = 0;
        try {

            if(sol_fail_count >= sol_fail_limit){
                cout << "Solution NOT Found" << endl;
                return 0;
            }

            // HOME THE TOOL AT HAND
            int index = -1;
            if(gripper != "") {
                auto it = std::find(gripper_list.begin(), gripper_list.end(), gripper);
                index = std::distance(gripper_list.begin(), it);
                if(is_aval_list[index] == false && is_tool_aval) {
                    homeTool(C, gripper, tool);
                    is_aval_list[index] = true;
                    cout << "Gripper is homing the tool" << endl;
                }
            }
           
            cout << "Step No: " << idx << "/" << path.d0 << endl;
            
            addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
            addMarker(C_base, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.1, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});

            if(view) C.view(false, "Waypoint");

            // MOVE THE TARGET OBJECT
            if(gripper != ""){
                std::shared_ptr<SolverReturn> r = moveSkeleton(C, gripper, target, interacted_target, contact_point, waypoint);
                if(r != nullptr) is_feas = r->eq - calib <= threshold;
            } else {
                is_feas = false;
            }
            cout << "Is feasible: " << is_feas << endl;

            // HANDLE THE FAIL CASES
            // If the path is not feasible, use the tool to reach the object, if the tool is not available, calibrate the robot as a workaround 
            if(!is_feas) {
                // CHANGE THE CONTACT POINT
                gripper = generateContactPoint(C, target, interacted_target, waypoint);
                std::shared_ptr<SolverReturn> r = moveSkeleton(C, gripper, target, interacted_target, contact_point, waypoint);
                if(r != nullptr) is_feas = r->eq - calib <= threshold;
                
                if(!is_feas) {

                    // CHECK IF A OBSTACLE IS IN THE WAY
                    if(is_dynamic) {
                        arr is_obstacle_exist = getCameraView(C, "cam", "dynamic_obstacle", filter);
                        if(is_obstacle_exist.d0 != 0) {
                            path.clear();
                            is_path_feas = generatePathPlan(path, qF);
                            if(!is_path_feas) return 0;
                            idx = 0;
                        }
                    }

                    // PUSH THE OBJECT WITH A TOOL
                    else if(is_tool_aval) {
                        tool = useTool(C, waypoint, target);
                        idx ++;
                        //calib = calibSkeleton(C);

                        std::ostringstream oss;
                        oss << C;
                        if(is_save_C) Cs.push_back(oss.str());

                        arr cp = C.getFrame(target.c_str())->getPosition();
                        cp.append(index);
                        cp.append(idx);
                        contact_points.append(cp);
                        
                    // HOME THE ROBOT AND CALIBRATE
                    } else {
                        sol_fail_count++;
                        cout << "Calibrating and homing the robot" << endl;
                        homeSkeleton(C);
                        calib = calibSkeleton(C);
                        cout << "Calibrated and homed the robot" << endl;
                    }   
                    gripper = generateContactPoint(C, target, interacted_target, waypoint); 
                }
            }    

            else {
                sol_fail_count = 0;

                std::ostringstream oss;
                oss << C;
                if(is_save_C) Cs.push_back(oss.str());

                arr cp = C.getFrame(contact_point.c_str())->getPosition();
                cp.append(index);
                cp.append(idx);
                contact_points.append(cp);
                idx ++;
            }

        } catch(...) {
            frame_err ++;
            std::cerr << "Frame error occured. Check this!" << std::endl;
            if(frame_err >= 10) {
                std::cerr << "Frame error limit reached. Exiting!" << std::endl;
                return 0;
            }
            gripper = findContactPoint(C, target, interacted_target, waypoint);
        }
    }
    cout << "The HMAP is completed."<< endl;
    return 1;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

bool HMAPBiman::generatePathPlan(arr& path_plan, arr goal) {

    std::vector<std::vector<int>> obstacle_combinations;
    // Generate the combinations of obstacles
    for (int i = 1; i < (1 << total_obstacle_count); ++i) {
        std::vector<int> combination;
        for (int j = 0; j < total_obstacle_count; ++j) {

            if (i & (1 << j)) {
                combination.push_back(j);
                cout << "Comb: " << j << endl; 
            }
        }

        obstacle_combinations.push_back(combination);
    }
    
    if(!is_path_given) {

        int combination_count = obstacle_combinations.size();

        // Continue to find a path until a feasible path is found by removing obstacles
        do { 
            cout << "Checking if path is available" << endl;
            is_path_aval = RRT(C2, path_plan, goal, view);

            if(!is_path_aval && total_obstacle_count == 0){
                cout << "RRT could not find a solution" << endl;
                return 0;
            }

            for (uint i = 0; i < total_obstacle_count; i++) {
                C2.getFrame(("obstacle_" + std::to_string(i)).c_str())->setContact(1);
            }
            
            if(!is_path_aval && total_obstacle_count != 0 && (obstacle_count < combination_count)) {

                obstacle_count++;
                is_path_blocked = true;
                cout << "Obstacle count: " << obstacle_count << endl;

                for (uint i = 0; i < obstacle_combinations[obstacle_count].size(); i++) {
                    int obs = obstacle_combinations[obstacle_count][i];
                    cout << "Eliminating: " << obs << endl;
                    C2.getFrame(("obstacle_" + std::to_string(obs)).c_str())->setContact(0);
                }
            }  


        } while (!is_path_aval && (obstacle_count <= combination_count) && total_obstacle_count != 0);
        
        if(!is_path_aval) {
            std::cerr << "No feasible path is available" << std::endl;
            return 0;
        }
        
    }
    
    // Move the obstacle to a feasible position
    if(is_path_blocked && obstacle_count > -1) {  
        cout << "REMOVING... "  << endl;
        for (uint i = 0; i < obstacle_combinations[obstacle_count].size(); i++) {
            is_feas = false;
            
            int obs = obstacle_combinations[obstacle_count][i];
            std::string obstacle = "obstacle_" + std::to_string(obs);
            addMarker(C, {q_obs(obs, 0), q_obs(obs, 1), q_obs(obs, 2)}, "waypoint_obs" , "world", 0.1,  false, {q_obs(obs, 3), q_obs(obs, 4), q_obs(obs, 5), q_obs(obs, 6)});

            std::string gripper = findContactPoint(C, obstacle, obstacle, "waypoint_obs");
            std::shared_ptr<SolverReturn> r = moveSkeleton(C, gripper, obstacle, obstacle, contact_point, "waypoint_obs");
            if(r != nullptr) is_feas = r->eq - calib <= threshold;

            if (!is_feas && is_tool_aval) {
                tool = useTool(C, "waypoint_obs" , obstacle);
            } 

            if (!is_feas && !is_tool_aval) {
                std::cerr << "Obstacle is not movable" << std::endl;
                return 0;
            }
        }
    }

    return 1;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::adjust_waypoint(rai::Configuration C2, std::string target, std::string waypoint_adj){
    KOMO komo(C2, 1, 1, 0, true); // one phase one time slice problem, with 'delta_t=1', order=0
    komo.addObjective({}, FS_positionDiff, {target.c_str(), waypoint_adj.c_str()}, OT_sos, {1e0}); // constraint: gripper position
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2}); // constraint: gripper position
    NLP_Solver solver(komo.nlp(), 2);
    auto ret = solver.solve();
    cout << "Move Cost: " <<ret->eq<< endl;
    arr n_waypoint = komo.getPath_qOrg()[0];
    return n_waypoint;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::set_dynamic_obs() {
    this->is_dynamic = true;
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
std::vector<std::string> HMAPBiman::getCs(){
    return Cs;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::setPath(arr path, bool is_blocked, int obstacle_count){
    this->path = path;
    this->is_path_blocked = is_blocked;
    this->is_path_given = true;
    this->obstacle_count = obstacle_count;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::setC(rai::Configuration C){
    this->C = C;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::setC2(rai::Configuration C2){
    this->C2 = C2;
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
bool HMAPBiman::RRT(rai::Configuration C2, arr& path, arr goal, bool view) {
    arr q0 =  C2.getJointState();
    std::cout << "Start State: " << q0 << " Final State: " << goal << std::endl;
    
    if(view){
        addMarker(C2, {q0(0), q0(1), q0(2)}, "q0", "world", 0.1, false, {q0(3), q0(4), q0(5), q0(6)});
        addMarker(C2, {goal(0), goal(1), goal(2)}, "qF", "world", 0.1, false, {goal(3), goal(4), goal(5), goal(6)});
        C2.view(true, "RRT");
    }

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

        bool isFeas = false;
        auto ret = std::make_shared<SolverReturn>();
        for (int i = 0; i < 30 && !isFeas; ++i) {
            cout << "RRT Iteration: " << i << endl;

            RRT_PathFinder rrt(*problem, q0, goal, 0.1/waypoint_factor);//rrt(*problem, q0, qF, 0.03, 8, 5000, 1);

            ret->time -= rai::cpuTime();
            arr temp_path = rrt.run();
            ret->time += rai::cpuTime();

            ret->feasible = temp_path.N;
            ret->x = temp_path;
            ret->evals = rrt.iters;

            isFeas = ret->feasible;
            path = ret->x;
        }
    } catch(...){
        std::cout << "Start State: " << q0 << " Final State: " << goal << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    
    std::cout << "Path Len: " << path.N << std::endl;
    if(path.N == 0) {
        std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    for (uint i = 0; i < path.d0; ++i) {
        C2.setJointState(path[i]);  // Use path[i] to access the i-th configuration in the path
        if(view) {
            addMarker(C2, {path(i,0), path(i,1), path(i,2)}, "cp" + std::to_string(i), "world", 0.1, false, {path(i,3), path(i,4), path(i,5), path(i,6)});
            C2.view(false);
            rai::wait(0.1);
        }
    }
    cout << "Path Length: " << path.d0 << endl; 
    return true;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::connectWaypoints(rai::Configuration C2, arr waypoints, bool view) {
    arr path = {};
    arr qF_s = qF;
    /*
    for(uint i = 0; i < waypoints.d0-1; i++){
        C2.setJointState(waypoints[i]);
        qF = waypoints[i+1];
        arr p = {};
        bool is_aval = RRT(C2, p, view);
        if(!is_aval) return {};
        path.append(p);
    }

    for (uint i = 0; i < path.d0; ++i) {
        C2.setJointState(path[i]); 
        if(view) {
            C2.view(false);
            rai::wait(0.5);
        }
    }

    qF = qF_s;
    */
    return path;
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

        

        if(pts_filtered.d0 == 0) {
            if(is_empty){
                return arr();
            }
            homeSkeleton(C);
            cout << "Homing for camera" << endl;
            is_empty = true;
        } else {
            pts_filtered.reshape(pts_filtered.d0/3,3);
            cout << "Filtered Points Length: " << pts_filtered.d0 << endl;
            if(pts_filtered.d0 <= 15){
                if(is_empty){
                    return arr();
                }
                homeSkeleton(C);
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
    }
    else {
        target_point = {pts(iter, 0), pts(iter,1), pts(iter, 2)};
    }

    return target_point;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

const std::string HMAPBiman::generateContactPoint(rai::Configuration& C, const std::string& target, const std::string& interacted_target, const std::string& waypoint){
    static int cp_count = 0;
    if(is_model_aval){
        // Give the model the configuration and retrieve the contact point
        arr cp_g = offline_cp_model[cp_count];
        addMarker(C, {cp_g(0), cp_g(1), cp_g(2)}, "contact_point", target, 0.1, false);
        std::string gripper = gripper_list[cp_g(3)];
        // Check if it is feasible
        std::shared_ptr<SolverReturn> r = moveSkeleton(C, gripper, target, interacted_target, contact_point, waypoint);
        bool f = false;
        if(r != nullptr) f = r->eq - calib <= threshold;
        if(f) return gripper;
        else return findContactPoint(C, target, interacted_target, waypoint);

    } else {
        return findContactPoint(C, target, interacted_target, waypoint);
    }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

const std::string HMAPBiman::findContactPoint(rai::Configuration& C, const std::string& target, const std::string& interacted_target, const std::string& waypoint){
    // Get the camera point cloud for the object
    static int cp_count = 0;

    arr pts = getCameraView(C, "cam", interacted_target, filter);

    arr sos_l_arr;
    arr sos_r_arr;
    int iter = 0;
    int max_iter;
    arr contactPoints;
    int fail_lim = 3;
    int fail = 0;
    int fail_count = 0;
 
    max_iter = 20;

    for (iter = 0; iter < max_iter && fail < fail_lim; iter++) {
        cout << "Iteration: " << iter << " / " << max_iter << endl;

        if (pts.d0 == 0){
            cout << "No candidate contact point found" << endl;
            return "";
        }   

        double pt_idx = rnd.num(0, pts.d0 - 1);
        arr c_pts = candidateContactPoint(C, pts, pt_idx, true);

        addMarker(C, c_pts, "candidate_point", target, 0.0001, false);

        int ret_fail = 0;
        double cost = threshold + 1;
        std::vector<double> cost_list = {};
        bool is_found = false;

        for (uint i = 0; i < gripper_count; i++){
            std::string g = gripper_list[i];
            bool is_timeout = false;
            std::shared_ptr<SolverReturn> ret = moveSkeletonWithTimeout(C, g, target, interacted_target, "candidate_point", waypoint.c_str(), true, 1, is_timeout);
            if(is_timeout) {
                ret_fail++;
            } 

            cost = ret->eq - calib;
            if(cost < threshold) {
                is_found = true;
            }
            
            cost_list.push_back(cost);
        }

        if(ret_fail == gripper_count && gripper_count != 1) {
            fail++;
            cout << "Fail No: " << fail  << "/" << fail_lim << endl;
        }

        if(is_found) {
            cout << "Found a feasible contact point" << endl;
            for (uint i = 0; i < gripper_count; i++){
                std::string g = gripper_list[i];
                auto min_it = std::min_element(cost_list.begin(), cost_list.end());
                int min_index = std::distance(cost_list.begin(), min_it);
                if (cost_list[i] < threshold && i == min_index) {
                    cout << "The " << g << " is closer to the object: " << cost_list[i] << endl;
                    contact_point = "contact_point_" + std::to_string(cp_count);
                    addMarker(C, c_pts, contact_point, target, 0.1, false);
                    cp_count++;
                    return g;
                }
            }
        }
    }
    cout << "Could not find a feasible contact point" << endl;
    return "";   
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/


double HMAPBiman::calibSkeleton(rai::Configuration& C) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;

    for (uint i = 0; i < gripper_count; i++){
        S.addEntry({1, -1}, rai::SY_poseEq, {(gripper_list[i]).c_str(), (gripper_list[i] + "_home").c_str()});
    }

    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();
    if(view) {
        cout <<komo_path->report(true, true, true) <<endl;
        C.view(true, "Calib Skeleton");
    }
    return ret->eq;
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::shared_ptr<SolverReturn> HMAPBiman::homeSkeleton(rai::Configuration& C) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;

    for(uint i = 0; i < gripper_count; i++){
        S.addEntry({1, -1}, rai::SY_poseEq, {(gripper_list[i]).c_str(), (gripper_list[i] + "_home").c_str()});
    }

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
    states.append(state);
    rai::Configuration C_v;
    C_v.copy(C);
    C_views.push_back(C_v);
    C.setFrameState(state);

    return ret;
} 

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::shared_ptr<SolverReturn> HMAPBiman::moveSkeletonWithTimeout(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& interacted_target, const std::string& candidate_point, const std::string& waypoint, bool flag, int timeout_seconds, bool& is_timeout) {
    // Wrap the moveSkeleton call in an async task
    auto future = std::async(std::launch::async, [&]() {
        return this->moveSkeleton(C, gripper, target, interacted_target, candidate_point, waypoint, flag);
    });
    
    // Wait for the result, but only for the timeout duration
    if (future.wait_for(std::chrono::seconds(timeout_seconds)) == std::future_status::timeout) {
        // If timeout occurs, return null or handle as appropriate
        std::cout << "moveSkeleton for " << gripper << " timed out." << std::endl;
        is_timeout = true;
    }
    
    // Otherwise, return the result of moveSkeleton
    return future.get();
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::shared_ptr<SolverReturn> HMAPBiman::moveSkeleton(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& interacted_target, const std::string& contact_point, const std::string& waypoint, const bool isTrial) {

    if(gripper == "" || target == "" || interacted_target == "" || contact_point == "" || waypoint == "") {
        std::cerr << "Error: Invalid input for moveSkeleton. Gripper: " << gripper << " Target: "  << target << " Interacted Target: " << interacted_target << " Contact Point: " << contact_point << " Waypoint: " << waypoint << std::endl;
        return nullptr;
    }
    

    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;    
    std::shared_ptr<KOMO> komo_path;

    S.addEntry({0.1, -1}, rai::SY_touch, {gripper.c_str(), interacted_target.c_str()});
    S.addEntry({0.5, -1}, rai::SY_stable, {gripper.c_str(), interacted_target.c_str()});
    S.addEntry({1, -1}, rai::SY_poseEq, {waypoint.c_str(), target.c_str()});
    komo_path = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    
    //komo_path->addObjective({1, -1}, FS_poseDiff, {target.c_str(), waypoint.c_str()}, OT_eq, {1e1});
    komo_path->addObjective({0.1, -1}, FS_positionDiff, {gripper.c_str(), contact_point.c_str()}, OT_sos, {1e1});
    //komo_path->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    cout << "Move Cost: " <<ret->eq - calib<< endl;
    
    if(!isTrial){
        if(view)cout <<komo_path->report(true, true, true) <<endl;

        if(((ret->eq - calib) <= threshold) && !isTrial) {
            if(view){
                komo_path->pathConfig.viewer()->raiseWindow();
                komo_path->view_play(false, 0.1, (video_path+ std::to_string(img_count)).c_str());
                komo_path->pathConfig.viewer()->close_gl();
                img_count++;
            }
            arr state = komo_path->pathConfig.getFrameState();
            state_all.push_back(komo_path);
            state_count++;
            double frame_count = komo_path->pathConfig.getFrameNames().d1;
            state = state.rows(state.d0-frame_count,state.d0);
            states.append(state);
            C.setFrameState(state);
            rai::Configuration C_v;
            C_v.copy(C);
            C_views.push_back(C_v);

            cout << "Waypoint: " << C.getFrame(waypoint.c_str())->getPose() << " Pose: " << C.getFrame("box")->getPose() <<  endl;
            if(view) C.view(true, "Move Skeleton");
            
            auto it = std::find(gripper_list.begin(), gripper_list.end(), gripper);
            int index = std::distance(gripper_list.begin(), it);
            is_aval_list[index] = true;
            cout << "End Move Skeleton" << endl;
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
    std::string tool_head = tool;

    size_t pos = tool_head.find("_handle");
    if (pos != std::string::npos) {
        tool_head.erase(pos, tool_head.length());
    }

    S.addEntry({0.7, -1}, rai::SY_touch, {gripper.c_str(), tool.c_str()});
    S.addEntry({1, -1}, rai::SY_stable, {gripper.c_str(), tool.c_str()});
    S.addEntry({1.3, -1}, rai::SY_touch, {tool_head.c_str(), target.c_str()});
    S.addEntry({1.6, -1}, rai::SY_stable, {tool_head.c_str(), target.c_str()});
    S.addEntry({1.9, -1}, rai::SY_poseEq, {target.c_str(), final_pose.c_str()});

    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e2);
    
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
        states.append(state);
        C.setFrameState(state);
        rai::Configuration C_v;
        C_v.copy(C);
        C_views.push_back(C_v);
        if(view) C.view(true, "Tool Skeleton");
        cout << "End Tool Skeleton" << endl;
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

    auto it = std::find(gripper_list.begin(), gripper_list.end(), gripper_out);
    int index = std::distance(gripper_list.begin(), it);
    is_aval_list[index] = false;

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
    states.append(state);
    C.setFrameState(state);
    rai::Configuration C_v;
    C_v.copy(C);
    C_views.push_back(C_v);
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::string HMAPBiman::useTool(rai::Configuration& C, const std::string waypoint, const std::string target){
    cout << "Tool use initiated!" << endl;
    std::string gripper_tool;
    std::string tool; 
    toolSelection(C, waypoint, target, gripper_tool, tool);
    toolSkeleton(C, tool, target, gripper_tool, waypoint);
    return tool;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void HMAPBiman::displaySolution(){
    rai::Configuration C_view;
    for(uint i = 0; i < state_count; i++){
        std::shared_ptr<KOMO> komo_path = state_all[i];
        C_view = C_views[i];
        arr state = komo_path->pathConfig.getFrameState();
        double frame_count = komo_path->pathConfig.getFrameNames().d1;

        for (uint j = 0; j < state.d0/frame_count-1; j++){
            state = state.rows(j*frame_count,(j+1)*frame_count);
            C_view.setFrameState(state);
            C_view.view(false, "Solution");
            rai::wait(0.1);
        }
        
        cout << i << "/" << state_count << endl;
    }
    return;
}

