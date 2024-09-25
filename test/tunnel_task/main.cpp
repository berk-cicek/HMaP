#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    rai::initCmdLine(argc, argv);
    rnd.seed(0);

    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../src/config/tunnel/HMAP_tunnel_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../src/config/tunnel/HMAP_tunnel_actuated_conf.g");  
    
    C.view(true, "Initial Configuration");

    std::string target = "box";
    std::vector<std::string> tool_list = {};
    std::vector<std::string> gripper_list = {"r_l_gripper", "l_l_gripper"};
    double filter = 0.06;
    std::string video_path = "video/config";
    int total_obstacle_count = 0;
    int waypoint_factor = 2;
    arr qF = {0.5, 0.25, 0.09001, 1, 0, 0, 0};
    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, {}, target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, true);
    hmap_biman.run();

    return 0;
}