#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    rai::initCmdLine(argc, argv);
    rnd.seed(0);

    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../HMAP/config/bookshelf_mini/HMAP_bookshelf_mini_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../HMAP/config/bookshelf_mini/HMAP_bookshelf_mini_actuated_conf.g");  
    
    C.view(true, "Initial Configuration");

    std::string target = "box";
    std::string interacted_target = "box";
    std::vector<std::string> tool_list = {};
    std::vector<std::string> gripper_list = {"l_l_gripper"};
    double filter = 1;
    std::string video_path = "video/config";
    int total_obstacle_count = 1;
    int waypoint_factor = 2;
    arr qF = {-0.20, -0.6, 0.615, 1, 0, 0, 0};
    arr q_obs = {0, -0.6, 0.82, 1, 0, 0, 0};
    q_obs.reshape(total_obstacle_count, 7);
    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, "cam_frame_0", 0);
    if(hmap_biman.run())
    hmap_biman.displaySolution();
    return 0;
}