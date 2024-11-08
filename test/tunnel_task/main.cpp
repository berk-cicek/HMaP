#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../HMAP/config/tunnel/HMAP_tunnel_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../HMAP/config/tunnel/HMAP_tunnel_actuated_conf.g");  
    
    C.view(true, "Initial Configuration");

    std::string target = "box";
    std::string interacted_target = "box";
    std::vector<std::string> tool_list = {};
    std::vector<std::string> gripper_list = {"r_l_gripper", "l_l_gripper"};
    double filter = 1;
    std::string video_path = "video/config";
    int total_obstacle_count = 0;
    int waypoint_factor = 2;
    arr qF = {0.5, 0.25, 0.09001, 1, 0, 0, 0};
    arr q_obs = {};
    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, false);
    hmap_biman.load_model("sample_tunnel/sample/keypoints_15");
    hmap_biman.is_model_aval = true;
    hmap_biman.run();
    //hmap_biman.displaySolution();


    return 0;
}