#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    rai::initCmdLine(argc, argv);
    rnd.seed(0);

    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("HMAP_tunnel_tool_env.g");  

    rai::Configuration C2;
    C2.addFile("HMAP_tunnel_tool_actuated_conf.g");  
    
    C.addFile(rai::raiPath("../rai-robotModels/panda/panda_tunnel.g"), "r_");
    C.addFile(rai::raiPath("../rai-robotModels/panda/panda_tunnel.g"), "l_");
    C.getFrame("l_panda_link0")->setPose(rai::Transformation(rai::Vector(0.3, -0.1, .05), rai::Quaternion(0.7073883, 0., 0., 0.7068252)));
    C.getFrame("r_panda_link0")->setPose(rai::Transformation(rai::Vector(-1, 0.4, .05), rai::Quaternion(1, 0., 0., 0)));

    C.view(true, "Initial Configuration");

    std::string target = "box";
    std::vector<std::string> tool_list = {"stick", "sphere", "cube"};
    std::vector<std::string> gripper_list = {"r_l_gripper", "l_l_gripper"};
    double filter = 0.06;
    std::string video_path = "video/";
    int total_obstacle_count = 0;
    arr qF = {-0.7, -0.4, .09001, 0, 0, 0, 1};
    arr q_obs = {};
    C2.setJointState(C.getFrame("box")->getPose());

    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, total_obstacle_count, tool_list, gripper_list, filter, video_path, true);
    hmap_biman.run();

    return 0;
}