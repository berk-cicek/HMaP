#include "HMAPBiman.h"

HMAPBiman::HMAPBiman(rai::Configuration C, rai::Configuration C2, arr qF, arr q_obs, std::string target, std::string interacted_target, int total_obstacle_count, std::vector<std::string> tool_list, std::vector<std::string> gripper_list, double filter, std::string video_path, int waypoint_factor, std::string cam, int verbose){
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
    this->point_clouds = {};
    this->verbose = verbose;
    this->video_path = video_path;
    this->fail_limit = 10;
    this->waypoint_factor = waypoint_factor;
    this->state_all = {};
    this->komo_waypoints = {};
    this->states = {};
    this->C_views = {};
    this->state_count = 0;
    this->is_dynamic = false;
    this->dyn_obs_idx = -1;
    this->dyn_obs_pos = {};
    this->cam = cam;

    if(tool_list.empty()) {
        this->is_tool_aval = false;
    } else {
        this->is_tool_aval = true;
    }
    this->Cs = {};

    for(int i = 0; i < gripper_count; i++){
        is_aval_list.push_back(true);
    }

}

bool HMAPBiman::run(){
    auto start = std::chrono::high_resolution_clock::now();

    if(verbose>1) C.view(true, "Initial Configuration");

    if(!tool_list.empty()){
        for (uint i = 0; i < tool_list.size(); i++){
            std::string t = tool_list[i];
            addMarker(C, C.getFrame(t.c_str())->getPosition(), (t + "_home"), "world", 0.001, false);
        }
    }
    
    for (int i = 0; i < gripper_count; i++){
        std::string g = gripper_list[i];
        addMarker(C, C.getFrame(g.c_str())->getPosition(), (g + "_home"), "world", 0.001, false, C.getFrame(g.c_str())->getQuaternion());
    }
    
    calib = calibSkeleton(C);
    bool is_found = false;
    int sol_fail_count = 0;
    int sol_try_count  = 0;
    int sol_retry_limit = 3;
    int sol_absolute_limit = 10;

    retry:
        idx = 0;
        // PATH PLANNING
        //cout << "Generating Path Plan" << endl;
        bool is_path_feas = generatePathPlan(path, qF);
        if(!is_path_feas) return 0;

        std::vector<std::string> robot_list;
        std::vector<std::string> target_list;
        std::vector<std::string> waypoint_list;
        std::vector<std::string> contact_point_list;

        std::string waypoint = "waypoint" + std::to_string(idx);

        // Check if there is enough clearance to reach the object
        addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.001, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
        
        // CHECK IF TARGET IS VISIBLE
        //cout << "Checking if object is reachable" << endl;

        std::string gripper = generateContactPoint(C, target, interacted_target, waypoint);
   
        if(gripper == ""){
            bool is_vis = getCameraView(C, cam, interacted_target, filter).d0 > 0;
            while(!is_vis && is_tool_aval && idx < path.d0) {
                waypoint = "waypoint" + std::to_string(rnd.uni(0, 10000));
                //cout << "Object is not reachable" << endl;
                addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.001, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
                tool = useTool(C, waypoint, target);
                is_vis = getCameraView(C, cam, interacted_target, filter).d0 > 0;
                idx++;
            }
        } else {
            idx++;
            waypoint = "waypoint" + std::to_string(rnd.uni(0, 10000));
            addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.001, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
        }

        if(idx == path.d0) {
            cout << "The HMAP is completed."<< endl;
            return 1;
        }

        std::string waypoint_opt = "";
        int time_step = 0;
        // THE MAIN LOOP
        // Now move the robot along the path

        bool is_tool_used = false;

        while(idx < path.d0 ) {
            cout << "Step No: " << idx << "/" << path.d0 << endl;
            waypoint = "waypoint" + std::to_string(rnd.uni(0, 1000));
            addMarker(C, {path(idx,0), path(idx,1), path(idx,2)}, waypoint, "world", 0.001, false, {path(idx,3), path(idx,4), path(idx,5), path(idx,6)});
            // ADD DYNAMIC OBSTACLE
            if(is_dynamic && idx == path.d0 / 3) {
                C.getFrame("dynamic_obstacle")->setPosition({path(idx+3*waypoint_factor,0), path(idx+3*waypoint_factor,1), path(idx+3*waypoint_factor,2)});
                C2.getFrame("dynamic_obstacle")->setPosition({path(idx+3*waypoint_factor,0), path(idx+3*waypoint_factor,1), path(idx+3*waypoint_factor,2)});
            } 

            int frame_err = 0;
            try {

                if(sol_fail_count >= sol_retry_limit){
                    if(sol_try_count >= sol_absolute_limit){
                        std::cerr << "Solution limit reached. Exiting!" << std::endl;
                        return 0;
                    } else {
                        sol_fail_count = 0;
                        C2.setJointState(C.getFrame(target.c_str())->getPose());
                        std::cerr << "Solution limit reached. Trying alternative path!" << std::endl;
                        goto retry;
                    }
                }

                // CHECK IF A OBSTACLE IS IN THE WAY
                if(is_dynamic) {
                    arr is_obstacle_exist = getCameraView(C, cam, "dynamic_obstacle", filter, -1);
                    if(is_obstacle_exist.d0 != 0) {
                        C2.setJointState(C.getFrame(target.c_str())->getPose());
                        if(total_obstacle_count != 0) C2.getFrame("obstacle_0")->setPosition(C.getFrame("obstacle_0")->getPosition());
                        path.clear();
                        is_path_feas = generatePathPlan(path, qF);
                        is_dynamic = false;
                        if(!is_path_feas) return 0;
                        idx = 0;
                    }
                }

                // HOME THE TOOL AT HAND
                int index = -1;
                if(gripper != "") {
                    auto it = std::find(gripper_list.begin(), gripper_list.end(), gripper);
                    index = std::distance(gripper_list.begin(), it);
                    if(is_aval_list[index] == false && is_tool_aval) {
                        homeTool(C, gripper, tool);
                        is_aval_list[index] = true;
                        //cout << "Gripper is homing the tool" << endl;
                    }
                }

                
            
                if(verbose>1) C.view(false, "Waypoint");

                // MOVE THE TARGET OBJECT
                if(gripper != "" && contact_point != "" && !is_tool_used) {
                    std::shared_ptr<SolverReturn> r = moveSkeleton(C, gripper, target, interacted_target, contact_point, waypoint);
                    if(r != nullptr) is_feas = r->eq - calib <= threshold;
                } else {
                    is_feas = false;
                }
                //cout << "Is feasible: " << is_feas << endl;

                // HANDLE THE FAIL CASES
                // If the path is not feasible, use the tool to reach the object, if the tool is not available, calibrate the robot as a workaround 
                if(!is_feas) {
                    // CHANGE THE CONTACT POINT
                    if(!is_tool_used)
                    gripper = generateContactPoint(C, target, interacted_target, waypoint);
                    if(gripper != "" && !is_tool_used) is_feas = true;
                    
                    if(!is_feas) {

                        // PUSH THE OBJECT WITH A TOOL
                        if(is_tool_aval) {
                            tool = useTool(C, waypoint, target);
                            idx +=1;
                            calib = calibSkeleton(C);
                            is_tool_used = true;

                        // HOME THE ROBOT AND CALIBRATE
                        } else {
                            sol_fail_count++;
                            sol_try_count++;
                            //cout << "Calibrating and homing the robot" << endl;
                            homeSkeleton(C);
                            calib = calibSkeleton(C);
                            //cout << "Calibrated and homed the robot" << endl;
                        }   
                        if(!is_tool_used)
                        gripper = generateContactPoint(C, target, interacted_target, waypoint); 
                        if(gripper != ""){
                            idx +=1; 
                        }   
                    }
                }    
                else {
    
                    idx +=1;

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


    auto end = std::chrono::high_resolution_clock::now();
    duration_all = end - start;
    cout << "The RRT is completed in " << duration_rrt.count() << "s" << endl;
    cout << "The HMAP is completed in " << duration_all.count() << "s" << endl;
    return 1;
    

}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void HMAPBiman::getTime(double& all, double& rrt){
    all = duration_all.count();
    rrt = duration_rrt.count();
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void HMAPBiman::load_model(std::string model_path){
    //C.view(true, "DASJh");
    //C2.view(true, "ASDLKj");
    is_model_aval = true;
    rai::Configuration C_n;
    C_n.addFile((model_path + "/config_0.g").c_str());
    StringA all_frames = C_n.getFrameNames();
    bool is_first = true;

    /*
    for(uint i = 0; i < all_frames.d0; i++){
        std::string name = all_frames(i).p;

        if(name == interacted_target.c_str()){
            arr js = C_n.getFrame(interacted_target.c_str())->getPosition();
            js.append({C_n.getFrame(interacted_target.c_str())->getQuaternion()*-1});
            C2.setJointState(js);
            C.getFrame(name.c_str())->setPosition(C_n.getFrame(name.c_str())->getPosition());
            C.getFrame(name.c_str())->setQuaternion(C_n.getFrame(name.c_str())->getQuaternion()*-1);
        }
        
        if(name.find("q_obs") != std::string::npos){
            if(is_first){
                q_obs.clear();
                is_first = false;
            }
            q_obs.append(C_n.getFrame(name.c_str())->getPosition());
            q_obs.append(C_n.getFrame(name.c_str())->getQuaternion());
            
        }

        if(name.find("r_") == std::string::npos && name.find("l_") == std::string::npos && name.find("stick") == std::string::npos &&
           name.find("cam") == std::string::npos && name.find("wp") == std::string::npos && name.find("q_obs") == std::string::npos &&
           name.find("waypoint") == std::string::npos && name.find("contact_point") == std::string::npos && name.find("candidate_point") == std::string::npos &&
           name.find("points") == std::string::npos && name.find(interacted_target.c_str()) == std::string::npos && name.find("pawn") == std::string::npos && name.find("obstacle") == std::string::npos) {

            C.getFrame(name.c_str())->setPosition(C_n.getFrame(name.c_str())->getPosition());
            C.getFrame(name.c_str())->setQuaternion(C_n.getFrame(name.c_str())->getQuaternion());
            C2.getFrame(name.c_str())->setPosition(C_n.getFrame(name.c_str())->getPosition());
            C2.getFrame(name.c_str())->setQuaternion(C_n.getFrame(name.c_str())->getQuaternion());
        }
    }

    q_obs.reshape(total_obstacle_count, 7);
    */

    //C.view(true, "Initial Configuration 2");
    int cp_count = 0;
    std::string wp_name;
    for (const auto& entry : std::filesystem::directory_iterator(model_path)) {
        if (std::filesystem::is_regular_file(entry.path())) {
            std::string filename = entry.path().filename().string();
            if (filename.find("pred") != std::string::npos) {
                cp_count++;
            } else if (filename.find("waypoint") != std::string::npos) {
                wp_name = filename;
            }
        }
    }

    for(int i = 0; i < cp_count; i++){
        arr cp = {};
        std::vector<double> row;
        std::string filename = model_path + "/pred_" + std::to_string(i) + ".csv";
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file " << filename << std::endl;
            return;
        }

        std::string line;

        if (std::getline(file, line)) {
            std::stringstream lineStream(line);
            std::string cell;

            while (std::getline(lineStream, cell, ',')) {
                try {
                    cp.append(std::stod(cell));
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid double value found in file: " << cell << std::endl;
                    continue;
                }
            }
        }
        file.close();
        offline_cp_model.append(cp);
    }

    std::ifstream file2(model_path + "/" + wp_name);
    if (!file2.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
    }

    arr p;

    std::string line;
    while (std::getline(file2, line)) {
        arr row;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            row.append(std::stod(value));
        }
        p.append(row);
    }

    file2.close();
    p.reshape(p.d0/7,7);
    bool is_obs = total_obstacle_count > 0;
    //C.getFrame(target.c_str())->setPosition({p(0,0), p(0, 1), p(0, 2)});
    //C.getFrame(target.c_str())->setQuaternion({p(0,3), p(0, 4), p(0, 5),  p(0, 6)});
    //idx = p.d0 - cp_count;
 
    rai::Configuration C_n2;
 
    //qF = {p(p.d0-1,0), p(p.d0-1, 1), p(p.d0-1, 2), p(p.d0-1,3), p(p.d0-1, 4), p(p.d0-1, 5),  p(p.d0-1, 6)};

    //setPath(p, is_obs, 0);
    offline_cp_model.reshape(cp_count, 4);

}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

bool HMAPBiman::generatePathPlan(arr& path_plan, arr goal) {
    is_path_blocked = false;
    obstacle_count = -1;
    std::vector<std::vector<int>> obstacle_combinations;
    // Generate the combinations of movable obstacles
    for (int i = 1; i < (1 << total_obstacle_count); ++i) {
        std::vector<int> combination;
        for (int j = 0; j < total_obstacle_count; ++j) {

            if (i & (1 << j)) {
                combination.push_back(j);
                //cout << "Comb: " << j << endl; 
            }
        }

        obstacle_combinations.push_back(combination);
    }
    
    auto start = std::chrono::high_resolution_clock::now();

    if(!is_path_given || !is_model_aval) {
        
        int combination_count = obstacle_combinations.size();
        
        // Continue to find a path until a feasible path is found by removing obstacles
        do { 
            //cout << "Checking if path is available" << endl;
            is_path_aval = RRT(C2, path_plan, goal);

            if(!is_path_aval && total_obstacle_count == 0){
                //cout << "RRT could not find a solution" << endl;
                return 0;
            }

            for (int i = 0; i < total_obstacle_count; i++) {
                C2.getFrame(("obstacle_" + std::to_string(i)).c_str())->setContact(1);
            }
            
            if(!is_path_aval && total_obstacle_count != 0 && (obstacle_count < combination_count)) {

                obstacle_count++;
                is_path_blocked = true;
                cout << "Obstacle count: " << obstacle_count << endl;

                for (uint i = 0; i < obstacle_combinations[obstacle_count].size(); i++) {
                    int obs = obstacle_combinations[obstacle_count][i];
                    //cout << "Eliminating: " << obs << endl;
                    C2.getFrame(("obstacle_" + std::to_string(obs)).c_str())->setContact(0);
                }
            }  

        } while (!is_path_aval && (obstacle_count <= combination_count) && total_obstacle_count != 0);
        
        if(!is_path_aval) {
            std::cerr << "No feasible path is available" << std::endl;
            return 0;
        }
        
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    duration_rrt = end - start;

    // Move the obstacle to a feasible position
    if(is_path_blocked && obstacle_count > -1) {  
        //cout << "REMOVING... "  << endl;
        for (uint i = 0; i < obstacle_combinations[obstacle_count].size(); i++) {
            is_feas = false;
            int obs = obstacle_combinations[obstacle_count][i];
            std::string obstacle = "obstacle_" + std::to_string(obs);
            std::string waypoint_obs = "waypoint_obs_" + std::to_string(rnd.uni(0, 1000));
            addMarker(C, {q_obs(obs, 0), q_obs(obs, 1), q_obs(obs, 2)}, waypoint_obs.c_str() , "world", 0.001,  false, {q_obs(obs, 3), q_obs(obs, 4), q_obs(obs, 5), q_obs(obs, 6)});

            std::string gripper = findContactPoint(C, obstacle, obstacle, waypoint_obs.c_str());
            
            if (gripper == "" && is_tool_aval) {
                tool = useTool(C, waypoint_obs.c_str() , obstacle);
            } 

            if (gripper == "" && !is_tool_aval) {
                std::cerr << "Obstacle is not movable" << std::endl;
                return 0;
            }
        }
    }
    is_path_blocked = 0;
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
    //cout << "Move Cost: " <<ret->eq<< endl;
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
    //this->obstacle_count = obstacle_count;
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
rai::Frame& HMAPBiman::addBox(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, arr size, bool is_relative, arr col, arr quat) {
    
    rai::Frame* point = C.addFrame(name.c_str(), parent.c_str());
    point->setShape(rai::ST_box, size);
    point->setColor(col);
    if(is_relative) {
        point->setRelativePosition(pos);
    } else {
        point->setPosition(pos);
    }
    if(quat.N == 0) {
        quat = C.getFrame(target.c_str())->getQuaternion();
    }
    point->setQuaternion(quat); // w, x, y, z
    return *point;
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
rai::Frame& HMAPBiman::addPoint(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr col, arr quat) {
    
    rai::Frame* point = C.addFrame(name.c_str(), parent.c_str());
    point->setShape(rai::ST_ssBox, {size, size, size, size});
    point->setColor(col);
    if(is_relative) {
        point->setRelativePosition(pos);
    } else {
        point->setPosition(pos);
    }
    if(quat.N == 0) {
        quat = C.getFrame(target.c_str())->getQuaternion();
    }
    point->setQuaternion(quat); // w, x, y, z
    return *point;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
bool HMAPBiman::RRT(rai::Configuration C2, arr& path, arr goal) {
    arr q0 =  C2.getJointState();
    if(verbose > 0) cout << "Start State: " << q0 << " Final State: " << goal << std::endl;
    if(verbose > 1){
        addMarker(C2, {q0(0), q0(1), q0(2)}, "q0", "world", 0.1, false, {q0(3), q0(4), q0(5), q0(6)});
        addMarker(C2, {goal(0), goal(1), goal(2)}, "qF", "world", 0.1, false, {goal(3), goal(4), goal(5), goal(6)});
        C2.view(true, "RRT");
    }

    auto problem = std::make_shared<ConfigurationProblem>(C2, true, 1e-10, 0);
    
    try {

        bool isFeas = false;
        auto ret = std::make_shared<SolverReturn>();
        for (int i = 0; i < 30 && !isFeas; ++i) {
            //cout << "RRT Iteration: " << i << endl;

            RRT_PathFinder rrt(*problem, q0, goal, 0.1/waypoint_factor);//rrt(*problem, q0, qF, 0.03, 8, 5000, 1);
            rrt.verbose = -1;

            ret->time -= rai::cpuTime();
            arr temp_path = rrt.run();
            ret->time += rai::cpuTime();

            ret->feasible = temp_path.N;
            ret->x = temp_path;
            ret->evals = rrt.iters;

            isFeas = ret->feasible;
            cout << isFeas << endl;
            path = ret->x;
        }
    } catch(...){
        //cout << "Start State: " << q0 << " Final State: " << goal << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    
    //cout << "Path Len: " << path.N << std::endl;
    if(path.N == 0) {
        //cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    //KOMO komo(C2, path.d0, 1, 2, true);
    //komo.initWithPath_qOrg(path);
    //komo.addControlObjective({0, path.d0}, 0, 1e-1);
    //komo.addControlObjective({0, path.d0}, 1, 1e-1);
    //komo.addControlObjective({0, path.d0}, 2, 1e-1);
    //komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2}); // constraint: gripper position
    //NLP_Solver solver(komo.nlp(), -1);
    //solver.solve();
    //path = komo.getPath_qOrg();

    if(verbose > 1) {
        for (uint i = 0; i < path.d0; i++) {
            C2.setJointState(path[i]);
            addMarker(C2, {path(i,0), path(i,1), path(i,2)}, "cp" + std::to_string(i), "world", 0.1, false, {path(i,3), path(i,4), path(i,5), path(i,6)});
            C2.view(false);
            rai::wait(0.1);
        }
    }
    C2.setJointState(q0);
    //cout << "Path Length: " << path.d0 << endl; 
    return true;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::getCameraView(rai::Configuration& C, const std::string& cam_name, const std::string& target, const double filter, int threshold) {
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
        rai::CameraView::Sensor sensor = V.addSensor(C.getFrame(cam_name.c_str()), 640, 360, 1.3, -1, {0.3, 5});

        pts_filtered.clear();
        V.computeImageAndDepth(img, depth);
        segmentation = V.computeSegmentationImage();

        uint target_frame_id = C.getFrame(target.c_str())->ID;
        arr target_frame_color = id2color(target_frame_id);
        uintA seg_id = V.computeSegmentationID();

        uint target_r = (uint)(target_frame_color(0) * 256);
        uint target_g = (uint)(target_frame_color(1) * 256);
        uint target_b = (uint)(target_frame_color(2) * 256);

        // Mask the image and depth data
        byteA masked_img = img;
        floatA depth_masked = depth;
        for (uint i = 0; i < segmentation.d0; i++) {
            for (uint j = 0; j < segmentation.d1; j++) {
                for (uint c = 0; c < 3; c++) {
                    if (segmentation(i, j, c) % 2 != 0) {
                        segmentation(i, j, c) = segmentation(i, j, c) + 1; // same as x+1 if x is odd
                    }
                }   
                bool matches = true;
                for (uint c = 0; c < 3; c++) {
                    double val = target_frame_color(c); 
                    byte tcol = (byte)std::round(val * 256.0);
                    if (segmentation(i, j, c) != tcol) {
                        matches = false;
                        break;
                    }
                }
                if (!matches) {
                    masked_img(i, j, 0) = 0;
                    masked_img(i, j, 1) = 0;
                    masked_img(i, j, 2) = 0;
                    depth_masked(i, j) = 0.0f;
                }
            }
        }

        if (verbose > 1){
            imgGl.watchImage(masked_img, true);
        }

        
        // Get the point cloud from the depth data and transform the points to the camera frame 
        depthData2pointCloud(pts, depth_masked, sensor.getFxycxy());

        //std::string frameName = "points_" + std::to_string(c);
        //rai::Frame* pts_frame = C.addFrame(frameName.c_str(), ("cam_frame_" + std::to_string(cam_frame_count)).c_str());

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        for (uint i = 0; i < pts.d0; i++) {
            for (uint j = 0; j < pts.d1; j++) {
                if (!(pts(i, j, 0) == 0 && pts(i, j, 1) == 0 && pts(i, j, 2) == 0)) {
                    if (dis(gen) <= filter) {
                        pts_filtered.append({pts(i,j,0), pts(i,j,1), pts(i,j,2)});
                    } 
                }
            }
        }

        if(pts_filtered.d0 == 0) {
            if(is_empty || threshold < 0){
                return arr();
            }
            homeSkeleton(C);
            //cout << "Homing for camera" << endl;
            is_empty = true;
        } else {
            if(threshold < 0) {
                return pts_filtered;
            }
            pts_filtered.reshape(pts_filtered.d0/3,3);
            //cout << "Filtered Points Length: " << pts_filtered.d0 << endl;
            if(pts_filtered.d0 <= threshold ){
                if(is_empty){
                    return arr();
                }
                homeSkeleton(C);
                //cout << "Homing for camera" << endl;
                is_empty = true;
            } else {
                //pts_frame->setPointCloud(pts_filtered, {0, 0, 0});
                //cout << pts_frame->getPosition() << endl;
                //addMarker(C, candidateContactPoint(C, pts_filtered, 2, true), "contact_pointads", "box", 0.1, false);
                //C.view(true, "Point cloud");
                is_empty = false;
            }

        }
    } while(is_empty);

    c++;
    //cout << pts_filtered.d0 << " " << pts_filtered.d1 << endl;
    return pts_filtered;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
arr HMAPBiman::candidateContactPoint(rai::Configuration& C, const arr& pts, const int iter, bool isTransform){
    if(pts.d0 == 0) {
        //cout << "No points found in the camera view" << std::endl;
        return {};
    }
    arr target_point;
    if(isTransform) {
        addMarker(C, {pts(iter, 0), pts(iter,1), pts(iter, 2)}, "target_point1", "cam_frame_0", 0.001, true);
        target_point = C.getFrame("target_point1")->getPosition();
        delete C.getFrame("target_point1");
    }
    else {
        target_point = {pts(iter, 0), pts(iter,1), pts(iter, 2)};
    }

    return target_point;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
const std::string HMAPBiman::generateContactPoint(rai::Configuration& C, const std::string& target, const std::string& interacted_target, const std::string& waypoint){
    static uint cp_count = 0;
    if(is_model_aval && cp_count < offline_cp_model.d0) {
        //cout << "Running the model for contact point" << endl;
        // Give the model the configuration and retrieve the contact point
        arr cp_g = offline_cp_model[cp_count];
        std::string contact_point_m = "contact_point_m" + std::to_string(rnd.num(0, 1000));
        addMarker(C, {cp_g(0), cp_g(1), cp_g(2)}, contact_point_m.c_str(), target, 0.001, true);

        std::string gripper = gripper_list[cp_g(3)];
        // Check if it is feasible
        std::shared_ptr<SolverReturn> r = moveSkeleton(C, gripper, target, interacted_target, contact_point_m.c_str(), waypoint);

        //C.view(true, "Contact Point Model");
        
        bool f = false;
        if(r != nullptr) f = r->eq - calib <= threshold;
        if(f) {
            cp_count ++;
            contact_point = contact_point_m;
            return gripper;
        }
        else{
            std::string gripper = findContactPoint(C, target, interacted_target, waypoint);
            if(gripper != "") cp_count ++;
            return gripper;
        }
    } else {
        std::string gripper = findContactPoint(C, target, interacted_target, waypoint);
        if(gripper != "") cp_count ++;
        return gripper;
    }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

const std::string HMAPBiman::findContactPoint(rai::Configuration& C, const std::string& target, const std::string& interacted_target, const std::string& waypoint){
    // Get the camera point cloud for the object
    static int cp_count = 0;

    arr pts = getCameraView(C, cam, interacted_target, filter);

    arr sos_l_arr;
    arr sos_r_arr;
    int iter = 0;
    int max_iter;
    arr contactPoints;
    int fail_lim = 3;
    int fail = 0;
    //int fail_count = 0;
 
    max_iter = 20;

    for (iter = 0; iter < max_iter && fail < fail_lim; iter++) {
        //cout << "Iteration: " << iter << " / " << max_iter << endl;

        if (pts.d0 == 0){
            //cout << "No candidate contact point found" << endl;
            return "";
        }   

        double pt_idx = rnd.num(0, pts.d0 - 1);
        arr c_pts = candidateContactPoint(C, pts, pt_idx, true);
        std::string candidate_point = "candidate_point_" + std::to_string(rnd.num(0, 1000));
        addMarker(C, c_pts, candidate_point.c_str(), target, 0.001, false);

        int ret_fail = 0;
        double cost = threshold + 1;
        std::vector<double> cost_list = {};
        bool is_found = false;

        for (int i = 0; i < gripper_count; i++){
            std::string g = gripper_list[i];
            bool is_timeout = false;
            
            std::shared_ptr<SolverReturn> ret = moveSkeletonWithTimeout(C, g, target, interacted_target, candidate_point.c_str(), waypoint.c_str(), true, 1, is_timeout);
            if(is_timeout) {
                ret_fail++;
            } 

            cost = ret->eq - calib;
            if(cost < threshold) {
                //cout << "Found a feasible contact point" << endl;
                //cout << "The " << g << " is closer to the object" << endl;
                contact_point = "contact_point_" + std::to_string(cp_count);
                addMarker(C, c_pts, contact_point, target, 0.001, false);
                cp_count++;
                return g;
            }
        }

        if(ret_fail && gripper_count != 1) {
            fail++;
            //cout << "Fail No: " << fail  << "/" << fail_lim << endl;
        }


    }
    //cout << "Could not find a feasible contact point" << endl;
    return "";   
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
double HMAPBiman::calibSkeleton(rai::Configuration& C) {
    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;

    for (int i = 0; i < gripper_count; i++){
        S.addEntry({1, -1}, rai::SY_poseEq, {(gripper_list[i]).c_str(), (gripper_list[i] + "_home").c_str()});
    }

    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();
    if(verbose > 2) {
        //cout <<komo_path->report(true, true, true) <<endl;
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

    for(int i = 0; i < gripper_count; i++){
        S.addEntry({1, -1}, rai::SY_poseEq, {(gripper_list[i]).c_str(), (gripper_list[i] + "_home").c_str()});
    }

    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 5, 1e-1, 1e-2, 1e-3, 1e1);
    std::shared_ptr<KOMO> komo_waypoint = S.getKomo_waypoints(C);

    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    //if(view){
    //    komo_path->pathConfig.view()->raiseWindow();
    //    komo_path->view_play(false, " ", 0.2, (video_path+ std::to_string(img_count)).c_str());
    //}

    img_count++;
    arr state = komo_path->pathConfig.getFrameState();

    state_all.push_back(komo_path);
    state_count ++;
    double frame_count = komo_path->pathConfig.getFrameNames().d1;
   
    state = state.rows(state.d0-frame_count,state.d0);
    states.append(state);
    komo_waypoints.push_back(komo_waypoint);
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
        //cout << "moveSkeleton for " << gripper << " timed out." << std::endl;
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
    

    auto it = std::find(gripper_list.begin(), gripper_list.end(), gripper);
    int index = std::distance(gripper_list.begin(), it);

    rai::Skeleton S;
    S.collisions = rai::getParameter<bool>("collisions", true);
    S.verbose = -1;    
    std::shared_ptr<KOMO> komo_path;
    std::shared_ptr<KOMO> komo_waypoint;
    double cp_ts = 0.1;

    if(!is_aval_list[index]) {
        S.addEntry({0.1, -1}, rai::SY_stable, {"table", tool.c_str()});
        S.addEntry({0.3, -1}, rai::SY_touch, {gripper.c_str(), interacted_target.c_str()});
        S.addEntry({0.5, -1}, rai::SY_stable, {gripper.c_str(), interacted_target.c_str()});
        S.addEntry({0.9, -1}, rai::SY_poseEq, {waypoint.c_str(), target.c_str()});  
        cp_ts = 0.3;
    } else {
        S.addEntry({0.1, -1}, rai::SY_touch, {gripper.c_str(), interacted_target.c_str()});
        S.addEntry({0.3, -1}, rai::SY_stable, {gripper.c_str(), interacted_target.c_str()});
        S.addEntry({0.8, -1}, rai::SY_poseEq, {target.c_str(), waypoint.c_str()});
    }

    komo_path = S.getKomo_path(C, 5, 1e-3, 1e-3, 1e-5, 1e2);
    komo_waypoint = S.getKomo_waypoints(C);

    komo_path->addObjective({cp_ts, -1}, FS_positionDiff, {gripper.c_str(), contact_point.c_str()}, OT_sos, {1e1});

    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    if(verbose > 1){
        cout << "Move Cost: " <<ret->eq - calib<< endl;
        cout <<komo_path->report(true, true, true) <<endl;
    }

    if(((ret->eq - calib) <= threshold)) {
        if(verbose > 1) {
            komo_path->view_play(false, " ", 0.1, (video_path+ std::to_string(img_count)).c_str());
            komo_path->view_close();
            img_count++;
        }
        arr state = komo_path->pathConfig.getFrameState();
        state_all.push_back(komo_path);
        rai::Configuration C_v;
        C_v.copy(C);
        C_views.push_back(C_v);
        state_count++;
        double frame_count = komo_path->pathConfig.getFrameNames().d1;
        state = state.rows(state.d0-frame_count,state.d0);
        states.append(state);
        C.setFrameState(state);

        komo_waypoints.push_back(komo_waypoint);
        //cout << "Waypoint: " << C.getFrame(waypoint.c_str())->getPose() << " Pose: " << C.getFrame("box")->getPose() <<  endl;
        if(verbose > 2) {C.view(true, "Move Skeleton");}
        
        is_aval_list[index] = true;
        //cout << "End Move Skeleton" << endl;
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

    S.addEntry({0.1, -1}, rai::SY_touch, {gripper.c_str(), tool.c_str()});
    S.addEntry({0.3, -1}, rai::SY_stable, {gripper.c_str(), tool.c_str()});
    S.addEntry({0.8, -1}, rai::SY_touch, {tool_head.c_str(), target.c_str()});
    S.addEntry({1.0, -1}, rai::SY_stable, {tool_head.c_str(), target.c_str()});
    S.addEntry({1.2, -1}, rai::SY_poseEq, {target.c_str(), final_pose.c_str()});

    std::shared_ptr<KOMO> komo_path  = S.getKomo_path(C, 10, 1e0, 1e-2, 1e-5, 1e3);
    //komo_path->addObjective({0.6, -1}, FS_quaternionDiff, { tool.c_str(), target.c_str()}, OT_sos, {1e3});
    //komo_path->addObjective({1, -1}, FS_quaternionDiff, { target.c_str(), final_pose.c_str()}, OT_sos, {1e3});
    //komo_path->addObjective({1.2, -1}, FS_poseDiff, {target.c_str(), final_pose.c_str()}, OT_sos, {1e3});
    //komo_path->addObjective({}, FS_accumulatedCollisions, {}, OT_sos, {1e1});
    std::shared_ptr<KOMO> komo_waypoint = S.getKomo_waypoints(C);
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    //cout << "Tool Cost: " << ret->eq << endl;

    if(!isTrial) {
        //if(view){
        //    komo_path->pathConfig.view()->raiseWindow();
        //    komo_path->view_play(false, " ", 0.2, (video_path+ std::to_string(img_count)).c_str());
        //    img_count++;
        //}
        arr state = komo_path->pathConfig.getFrameState();
        state_all.push_back(komo_path);
        state_count++;
        double frame_count = komo_path->pathConfig.getFrameNames().d1;
        state = state.rows(state.d0-frame_count,state.d0);
        states.append(state);

        komo_waypoints.push_back(komo_waypoint);
        rai::Configuration C_v;
        C_v.copy(C);
        C_views.push_back(C_v);
        C.setFrameState(state);
        if(verbose > 2) C.view(true, "Tool Skeleton");
        //cout << "End Tool Skeleton" << endl;
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
            //cout << "Tool: " << tool << " Gripper: " << gripper << endl;
            double cost = toolSkeleton(C, tool, target, gripper, waypoint, true);
            cost_arr[i][j] = cost;
        }
    }   

    double min_value = std::numeric_limits<double>::max(); // Set to a large value initially
    int min_row = -1;
    int min_col = -1;
    for (uint i = 0; i < cost_arr.size(); ++i) {
        for (uint j = 0; j < cost_arr[i].size(); ++j) {
            if (cost_arr[i][j] < min_value) {
                min_value = cost_arr[i][j];
                min_row = i;
                min_col = j;
            }
        }
    }
    tool_out = tool_list[min_row];
    gripper_out = gripper_list[min_col];

    //cout << "Best Tool: " << tool_out << endl;
    //cout << "Best Gripper: " << gripper_out << endl;
    this->tool = tool_out;
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
    std::shared_ptr<KOMO> komo_waypoint = S.getKomo_waypoints(C);
    NLP_Solver sol;
    sol.opt.verbose = -1;
    sol.setProblem(komo_path->nlp());
    auto ret = sol.solve();

    //if(view){
    //    komo_path->pathConfig.view()->raiseWindow();
    //    komo_path->view_play(false, " ", 0.2, (video_path + std::to_string(img_count)).c_str());
    //    img_count++;
    //}

    arr state = komo_path->pathConfig.getFrameState();
    state_all.push_back(komo_path);
    state_count++;
    double frame_count = komo_path->pathConfig.getFrameNames().d1;
    state = state.rows(state.d0-frame_count,state.d0);
    states.append(state);

    komo_waypoints.push_back(komo_waypoint);
    rai::Configuration C_v;
    C_v.copy(C);
    C_views.push_back(C_v);
    C.setFrameState(state);
} 
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::string HMAPBiman::useTool(rai::Configuration& C, const std::string waypoint, const std::string target){
    //cout << "Tool use initiated!" << endl;
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

    for(int i = 0; i < state_count; i++){
        std::shared_ptr<KOMO> komo_path = state_all[i];
        C_view = C_views[i];
        if(i == 0) {C_view.view(true, "Solution");}

        arr state = komo_path->pathConfig.getFrameState();
        double frame_count = komo_path->pathConfig.getFrameNames().d1;
        int v_count = state.d0/frame_count-1;

        for (int j = 0; j < v_count; j++) {
            arr frame_state = state.rows(j*frame_count, (j+1)*frame_count);
            C_view.setFrameState(frame_state);
            C_view.view(false);
            rai::wait(0.1);
        }
   
    }
} 



