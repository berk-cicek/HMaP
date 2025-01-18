#ifndef __HMAPBIMAN_H
#define __HMAPBIMAN_H

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
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <future>
#include <memory>
#include <chrono>
#include <filesystem>

class HMAPBiman {
public:
    HMAPBiman(rai::Configuration C, rai::Configuration C2, arr qF, arr q_obs, std::string target, std::string interacted_target, int total_obstacle_count, std::vector<std::string> tool_list, std::vector<std::string> gripper_list, double filter, std::string video_path, int waypoint_factor, std::string cam, int verbose);
    bool run();
    arr getPath();
    std::vector<std::string> getCs();
    void displaySolution();
    void setPath(arr path, bool is_blocked, int obstacle_count);
    void getTime(double& all, double& rrt);
    rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat = {});
    void setC(rai::Configuration C);
    void setC2(rai::Configuration C2);
    bool RRT(rai::Configuration C2, arr& path, arr goal);
    void set_dynamic_obs();
    void load_model(std::string model_path);
    
    bool is_save_C;
    bool is_model_aval;
private:
    struct Quaternion {
        double w, x, y, z;
    };
    rai::Configuration C;
    rai::Configuration C2;
    rai::Configuration C_copy;
    arr qHome;
    arr qF;
    arr q_obs;
    arr path;
    arr contact_points;
    arr dyn_obs_pos;
    arr offline_cp_model;
    arr states;
    arr point_clouds;
    bool is_dynamic;
    bool is_path_given;
    bool is_tool_aval;
    bool is_feas;
    bool is_path_aval;
    bool is_path_blocked;
    int verbose;
    int dyn_obs_idx;
    int idx;
    int gripper_count;
    int obstacle_count;
    int total_obstacle_count;
    int img_count;
    int fail_limit;
    int waypoint_factor;
    double calib;
    double threshold;
    double filter;
    int state_all_d0;
    int state_all_d1;
    int state_all_d2;
    int f_count;
    int state_count;
    std::string cam;
    std::string tool;
    std::string target;
    std::string interacted_target;
    std::string contact_point; 
    std::string video_path;
    std::vector<std::string> tool_list;
    std::vector<std::string> gripper_list;
    std::vector<std::shared_ptr<KOMO>> state_all;
    std::vector<std::shared_ptr<KOMO>> komo_waypoints;
    std::vector<bool> is_aval_list;
    std::vector<std::string> Cs;
    std::vector<rai::Configuration> C_views;
    std::chrono::duration<double> duration_rrt;
    std::chrono::duration<double> duration_all;

    rai::Frame& addBox(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, arr size, bool is_relative, arr col, arr quat = {});
    rai::Frame& addPoint(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr col, arr quat = {});
    arr getCameraView(rai::Configuration& C, const std::string& cam_name, const std::string& target, const double filter = 0.5, int threshold = 15);
    arr candidateContactPoint(rai::Configuration& C, const arr& pts, const int iter, bool isTransform = true);
    const std::string findContactPoint(rai::Configuration& C, const std::string& target, const std::string& interacted_target, const std::string& waypoint);
    const std::string generateContactPoint(rai::Configuration& C, const std::string& target, const std::string& interacted_target, const std::string& waypoint);
    double calibSkeleton(rai::Configuration& C);
    std::shared_ptr<SolverReturn> homeSkeleton(rai::Configuration& C);
    std::shared_ptr<SolverReturn> moveSkeleton(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& interacted_target, const std::string& contact_point, const std::string& waypoint, const bool isTrial=false);
    std::shared_ptr<SolverReturn> moveSkeletonWithTimeout(rai::Configuration& C, const std::string& gripper, const std::string& target, const std::string& interacted_target, const std::string& candidate_point, const std::string& waypoint, bool flag, int timeout_seconds, bool& is_timeout);
    double toolSkeleton(rai::Configuration& C, const std::string& tool, const std::string& target, const std::string& gripper, const std::string& final_pose, const bool isTrial = false);
    void toolSelection(rai::Configuration& C, const std::string waypoint, const std::string target, std::string& gripper_out, std::string& tool_out);
    void homeTool(rai::Configuration& C, const std::string& gripper, const std::string& tool);
    std::string useTool(rai::Configuration& C, const std::string waypoint, const std::string target);
    arr adjust_waypoint(rai::Configuration C2, std::string target, std::string waypoint_adj);
    bool generatePathPlan(arr& path_plan, arr goal);
    
};

#endif