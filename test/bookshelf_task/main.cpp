#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    rai::initCmdLine(argc, argv);
    rnd.seed(0);

    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../HMAP/config/bookshelf/HMAP_bookshelf_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../HMAP/config/bookshelf/HMAP_bookshelf_actuated_conf.g");  
    
    std::string target = "box";
    std::string interacted_target = "box";
    std::vector<std::string> tool_list = {"stick"};
    std::vector<std::string> gripper_list = {"r_l_gripper", "l_l_gripper"};
    double filter = 1;
    std::string video_path = "video/config";
    int total_obstacle_count = 1;
    int waypoint_factor = 2;
    arr qF = {0.20, -0.57, 1.29, 1, 0, 0, 0};
    arr q_obs = {{-0.19, -0.4, 0.95, 1, 0, 0, 0}};
    q_obs.reshape(total_obstacle_count, 7);
    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, true);
    hmap_biman.run();
    hmap_biman.displaySolution();
    return 0;
}