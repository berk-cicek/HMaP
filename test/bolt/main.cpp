#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../src/config/bolt/HMAP_bolt_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../src/config/bolt/HMAP_bolt_actuated_conf.g");  
    
    C.view(true, "Initial Configuration");

    std::string target = "head";
    std::string interacted_target = "head";
    std::vector<std::string> tool_list = {};
    std::vector<std::string> gripper_list = {"l_l_gripper"};
    double filter = 1;
    std::string video_path = "video/config";
    int total_obstacle_count = 0;
    int waypoint_factor = 2;
    arr qF = {-0.1, 0.4, .12, 1, 0, 0, 0};
    arr q_obs = {};
    C2.setJointState(C.getFrame("head")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, "cam_frame_0", 0);
    if(hmap_biman.run()){
        hmap_biman.displaySolution();
    }
    

    return 0;
}