#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../src/config/puck_obs_move/HMAP_puck_obs_move_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../src/config/puck_obs_move/HMAP_puck_obs_move_actuated_conf.g");  

    C.view(true, "Initial Configuration");

    std::string target = "box";
    std::string interacted_target = "pawn_handle";
    std::vector<std::string> tool_list = {"pawn_handle"};
    std::vector<std::string> gripper_list = {"l_l_gripper"};
    double filter = 1;
    std::string video_path = "video/config";
    int total_obstacle_count = 3;
    int waypoint_factor = 2;
    arr qF = {-0.26, -0.2, .1, 1, 0, 0, 0};
    arr q_obs;
    q_obs.append({-0.3, -0.08, .08, 1, 0, 0, 0});
    q_obs.append({-0.26, -0.08, .08, 1, 0, 0, 0});
    q_obs.append({-0.22, -0.08, .08, 1, 0, 0, 0});
    q_obs.reshape(total_obstacle_count, 7);

    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, "cam_frame_0", 0);
    if(hmap_biman.run())
    hmap_biman.displaySolution();



    return 0;
}