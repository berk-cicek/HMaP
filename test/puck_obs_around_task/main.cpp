#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../HMAP/config/puck_obs_around/HMAP_puck_obs_around_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../HMAP/config/puck_obs_around/HMAP_puck_obs_around_actuated_conf.g");  
    
    C.view(true, "Initial Configuration");

    std::string target = "box";
    std::string interacted_target = "pawn_handle";
    std::vector<std::string> tool_list = {"pawn_handle"};
    std::vector<std::string> gripper_list = {"l_l_gripper"};
    double filter = 0.001;
    std::string video_path = "video/config";
    int total_obstacle_count = 0;
    int waypoint_factor = 1;
    arr qF = {-0.35, -0.2, .06, 1, 0, 0, 0};
    arr q_obs = {};
    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, false);
    hmap_biman.run();
    hmap_biman.displaySolution();



    return 0;
}