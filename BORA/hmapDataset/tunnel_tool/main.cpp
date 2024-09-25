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
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int img_count = 0;
struct Quaternion {
    double w, x, y, z;
};
rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat = {0.7, 0, 0, 0.7});
void randomizeBox(rai::Configuration& C, rai::Configuration& C2);
void randomizeTunnel(rai::Configuration& C, rai::Configuration& C2, StringA& all_frames);
void randomizeBin(rai::Configuration& C, rai::Configuration& C2, StringA& all_frames);
double degreesToRadians(double degrees);
Quaternion eulerToQuaternion(double roll, double pitch, double yaw);
bool RRT(rai::Configuration& C2, const arr& qF, arr& path, arr& contact_points, bool view = true, double rrt_extend_length = 0.04);
void getCameraView(rai::Configuration& C, const std::string& cam_frame, byteA& img, floatA& depth, arr& pts);
void serialize_img(const byteA& img, const floatA& depth, const std::string& filename);
void serialize_csv(const std::map<std::string, arr>& dictionary, const std::string& filename);
void serialize_csv(const std::vector<std::map<std::string, std::string>>& conf, const std::string& filename);
void serialize_csv(const arr& array, const std::string& filename);
void serialize_C(const rai::Configuration& C, const std::string& filename);
std::string to_string(const arr& array);
std::string to_string(const rai::Shape& shape);
void create_dir(const std::string& filepath);
/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    rai::initCmdLine(argc, argv);
    rnd.seed(0);

    // Initialize Configuration and BotOp
    rai::Configuration C;
    
    rai::Configuration C_copy;
    C.addFile("../../HMAP/config/tunnel_tool/HMAP_tunnel_tool_conf.g");  
    C_copy.addFile("../../HMAP/config/tunnel_tool/HMAP_tunnel_tool_conf.g"); 

    rai::Configuration C2;
    rai::Configuration C2_copy;
    C2.addFile("../../HMAP/config/tunnel_tool/HMAP_tunnel_tool_actuated_conf.g");  
    C2_copy.addFile("../../HMAP/config/tunnel_tool/HMAP_tunnel_tool_actuated_conf.g");

    C.view(true, "Initial Configuration");

    StringA all_frames = C.getFrameNames();
    std::vector<std::string> tool_list = {"stick", "sphere", "cube"};
    uint count = 0;
    uint sample_count = 512;
    std::map<std::string, arr> dictionary_cam;

    // Get camera info
    for (uint i = 0; i < 3; i++){
        rai::Frame* cam_frame = C.getFrame(("cam_frame_" + std::to_string(i)).c_str());
        dictionary_cam["camera" + std::to_string(i) + "_position"] = cam_frame->getPosition();
        dictionary_cam["camera" + std::to_string(i) + "_rotation"] = cam_frame->getRotationMatrix();
    }

    for(uint i = 0; i < sample_count; i++) {
        C = C_copy;
        C2 = C2_copy;
        arr path;
        arr contact_points;

        std::string base_name = "sample/tunnel_tool_" + std::to_string(count) + "/";
        arr qF = {-0.7, -0.4, .09001, 1, 0, 0, 0};

        addMarker(C, {qF(0), qF(1), qF(2)}, "wp", "bin", 0.2, false, {qF(3), qF(4), qF(5), qF(6)});

        // Randomize box
        randomizeBox(C, C2);

        // Randomize the tunnel
        randomizeTunnel(C, C2, all_frames);

        // Randomize the bin
        randomizeBin(C, C2, all_frames);

        // Randomize tool
        for(const auto& t : tool_list){
            rai::Frame* tool = C.getFrame(t.c_str());
            arr xyz_tool = tool->getPosition();
            xyz_tool += {rnd.uni(-0.1, 0.1), 0, 0};
            tool->setPosition(xyz_tool);
            tool->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
        }

        C2.setJointState(C.getFrame("box")->getPose());
        qF = C.getFrame("wp")->getPose();
        
        // Generate waypoints
        if(RRT(C2, qF, path, contact_points, false)){
            serialize_csv(dictionary_cam, base_name + "cam_info.csv");
            serialize_csv(path, base_name + "waypoint_"+std::to_string(count) + ".csv");
            serialize_csv(contact_points, base_name + "contact_point_"+std::to_string(count) + ".csv");

            for(uint j = 0; j < 3; j++){
                byteA img;
                floatA depth;
                arr pts;
                getCameraView(C, "cam_frame_"+std::to_string(j), img, depth, pts);
                serialize_img(img, depth, base_name + "angle_" + std::to_string(j));
                serialize_csv(pts, base_name + "point_cloud_" + std::to_string(j) + ".csv");
            }
            
            serialize_C(C, base_name + "config_"+std::to_string(count) + ".g");
            
            count++;
        }
    }
    cout << "FEASIBLE COUNT: " << count << endl;
    return 0;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------FUNCTIONS------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat) {
    rai::Frame* marker = C.addFrame(name.c_str(), parent.c_str());
    marker->setShape(rai::ST_marker, {size, size, size, size});
    if(is_relative) {
        marker->setRelativePosition(pos);
    } else {
        marker->setPosition(pos);
    }
    marker->setQuaternion(quat); // w, x, y, z
    return *marker;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void randomizeBox(rai::Configuration& C, rai::Configuration& C2){
    rai::Frame* box = C.getFrame("box");
    rai::Frame* box2 = C2.getFrame("box");
    rai::Frame* cp   = C2.getFrame("contact_point");
    arr xyz_box = box->getPosition();
    arr size_box = box->getSize();
    double scale_box = rnd.uni(0.7, 1);
    arr pos_dif = {rnd.uni(-0.1, 0.1), 0, 0};
    xyz_box += pos_dif;
    size_box *= {scale_box, scale_box, 1, 1};
    arr cp_pos = cp->getRelativePosition() + pos_dif*1.5 ;
    cp->setRelativePosition(cp_pos);
    box->setPosition(xyz_box);
    box->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
    box->setShape(rai::ST_ssBox, size_box);
    box2->setPosition(xyz_box);
    box2->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
    box2->setShape(rai::ST_ssBox, size_box);
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void randomizeTunnel(rai::Configuration& C, rai::Configuration& C2, StringA& all_frames){
        rai::Frame* tunnel = C.getFrame("tunnel");
        rai::Frame* tunnel2 = C2.getFrame("tunnel");
        arr rpy_tunnel = {0, 0, rnd.uni(-10, 10)};
        arr xyz_tunnel = {rnd.uni(-0.1, 0.1), rnd.uni(-0.1, 0.1), 0};
        Quaternion q_tunnel = eulerToQuaternion(rpy_tunnel(0), rpy_tunnel(1), rpy_tunnel(2));
        tunnel->setQuaternion({q_tunnel.w, q_tunnel.x, q_tunnel.y, q_tunnel.z});    
        tunnel->setPosition(xyz_tunnel);
        tunnel2->setQuaternion({q_tunnel.w, q_tunnel.x, q_tunnel.y, q_tunnel.z});    
        tunnel2->setPosition(xyz_tunnel);
        double scale_tunnel = rnd.uni(0.9, 1.1);
        
        for (const auto& frame : all_frames){
            if (frame.contains("tunnel_")) {
                rai::Frame* obj = C.getFrame(frame);
                rai::Frame* obj2 = C2.getFrame(frame);
                arr size_tunnel = obj->getSize();
                size_tunnel *= {scale_tunnel, scale_tunnel, scale_tunnel, 1};
                obj->setShape(rai::ST_box, size_tunnel);
                obj->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
                obj2->setShape(rai::ST_box, size_tunnel);
                obj2->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
            }
        }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void randomizeBin(rai::Configuration& C,rai::Configuration& C2, StringA& all_frames){
    rai::Frame* bin = C.getFrame("bin");
    rai::Frame* bin2 = C2.getFrame("bin");
    arr box_q = C.getFrame("box")->getQuaternion();
    arr rpy_bin = {0, 0, rnd.uni(-10, 10)};
    arr xyz_bin = {rnd.uni(-0.1, 0.1), rnd.uni(-0.1, 0.1), 0};
    Quaternion q_bin = eulerToQuaternion(rpy_bin(0), rpy_bin(1), rpy_bin(2));
    bin->setQuaternion({q_bin.w, q_bin.x, q_bin.y, q_bin.z});    
    bin->setPosition(xyz_bin);
    bin2->setQuaternion({q_bin.w, q_bin.x, q_bin.y, q_bin.z});    
    bin2->setPosition(xyz_bin);

    double scale_bin = rnd.uni(0.9, 1.1);
    
    for (const auto& frame : all_frames){
        if (frame.contains("bin_")) {
            rai::Frame* obj = C.getFrame(frame);
            rai::Frame* obj2 = C2.getFrame(frame);
            arr size_bin = obj->getSize();
            size_bin *= {scale_bin, scale_bin, scale_bin, 1};
            obj->setShape(rai::ST_box, size_bin);
            obj->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
            obj2->setShape(rai::ST_box, size_bin);
            obj2->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
        }
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    // Convert angles from degrees to radians
    roll = degreesToRadians(roll);
    pitch = degreesToRadians(pitch);
    yaw = degreesToRadians(yaw);

    // Compute the quaternion components
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
bool RRT(rai::Configuration& C2, const arr& qF, arr& path, arr& contact_points, bool view, double rrt_extend_length) {
    arr q0 = C2.getJointState();
    std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
    addMarker(C2, {q0(0), q0(1), q0(2)}, "cp0", "world", 0.2, false, {q0(3), q0(4), q0(5), q0(6)});
    addMarker(C2, {qF(0), qF(1), qF(2)}, "cp", "world", 0.2, false, {qF(3), qF(4), qF(5), qF(6)});
    C2.view(true, "RRT");
    auto problem = std::make_shared<ConfigurationProblem>(C2, true, 1e-3, 0);
    auto collisionPairs = rai::getParameter<StringA>("collisionPairs", {});
    auto coll = C2.getCollidablePairs();
    StringA collisionPairsString;
    for (const auto& frame : coll) {
        collisionPairsString.append(frame->name);
    }
    std::cout << coll.N <<  std::endl;
    if(collisionPairs.N) {
        problem->setExplicitCollisionPairs(collisionPairsString);
    }
    
    try {
        RRT_PathFinder rrt(*problem, q0, qF);
        rrt.verbose = -1;
        bool isFeas = false;
        auto ret = std::make_shared<SolverReturn>();
        for (int i = 0; i < 10 && !isFeas; ++i) {
            //rrt = std::make_shared<RRT_PathFinder>(*problem, q0, qF, rrt_extend_length);
            RRT_PathFinder rrt2(*problem, q0, qF);
            rrt2.verbose = -1;
            // BU KISIM EKSTRA - RRT_PathFinder.cpp içinde PathFinder::solve kodundan
            ret->time -= rai::cpuTime();
            arr temp_path = rrt2.run();
            ret->time += rai::cpuTime();
            // BU KISIM EKSTRA - RRT_PathFinder.cpp içinde PathFinder::solve kodundan

            // BU KISIM GEREKLİ - RRT_PathFinder.cpp içinde PathFinder::solve kodundan
            ret->feasible = temp_path.N;
            ret->x = temp_path;
            ret->evals = rrt2.iters;

            isFeas = ret->feasible;
            path = ret->x;
        }
    } catch(...){
        std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    
    std::cout << "Path Prev: " << path.N << std::endl;
    if(path.N == 0) {
        std::cout << "Start State: " << q0 << " Final State: " << qF << std::endl;
        std::cerr << "Error: Path is not feasible!" << std::endl;
        return false;
    }

    KOMO komo2(C2, path.d0, 1, 2, true);
    komo2.addControlObjective({}, 2, 1e0);
    komo2.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});
    komo2.initWithPath_qOrg(path);

    NLP_Solver solver(komo2.nlp(), -1);
    solver.setOptions(rai::OptOptions().set_stopTolerance(1e-2));
    solver.solve();

    path = komo2.getPath_qOrg();

    // Interpolate the path by increasing the size by twice
    arr newPath;
    for (uint i = 0; i < path.d0 - 1; ++i) {
        newPath.append(path[i]);
        newPath.append((path[i] + path[i + 1]) / 2);
    }

    newPath.resize(newPath.d0/path.d1, path.d1);

    for (uint i = 0; i < newPath.d0; ++i) {
        C2.setJointState(newPath[i]);  // Use path[i] to access the i-th configuration in the path
        arr cp = C2.getFrame("contact_point")->getPosition();
        cp.append(1);
        contact_points.append(cp);
        if(view) {
            C2.view(false);
            rai::wait(0.1);
        }
    }
    contact_points.reshape(contact_points.d0/4, 4);
    path = newPath;
    
    return true;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void getCameraView(rai::Configuration& C, const std::string& cam_frame, byteA& img, floatA& depth, arr& pts){
    rai::CameraView V(C, true);
    OpenGL imgGl;
    rai::CameraView::Sensor sensor = V.addSensor("cam", (cam_frame).c_str(), 640, 360, 1.3, -1, {0.3, 5});

    // Get image and depth data from camera
    V.computeImageAndDepth(img, depth);
    arr D = rai::convert<double>(depth);
    depthData2pointCloud(pts, depth, sensor.getFxycxy());

    //imgGl.watchImage(img, true);
    //imgGl.watchImage(depth, true);
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void serialize_img(const byteA& img, const floatA& depth, const std::string& filename) {
    create_dir(filename);

    int numChannels = img.d2;  // Adjust based on how your image stores channels

    // Save the color image as PNG
    std::string img_filename = filename + "_image.png";
    stbi_write_png(img_filename.c_str(), img.d1, img.d0, numChannels, img.p, img.d1 * numChannels);

    // Compute maximum depth value
    float maxDepth = 0.0f;
    for (uint row = 0; row < depth.d0; ++row) {
        for (uint col = 0; col < depth.d1; ++col) {
            float value = depth(row, col);  // Access depth value
            if (value > maxDepth) {
                maxDepth = value;
            }
        }
    }

    // Normalize depth data for saving
    std::vector<uint8_t> depthNormalized(depth.d0 * depth.d1);  // Adjust size based on depth dimensions

    for (size_t i = 0; i < depth.d0 * depth.d1; ++i) {
        float depthValue = depth(i / depth.d1, i % depth.d1);  // Access depth value
        depthNormalized[i] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, depthValue * (255.0f / maxDepth))));
    }

    // Save the normalized depth image as PNG
    std::string depth_filename = filename + "_depth.png";
    stbi_write_png(depth_filename.c_str(), depth.d1, depth.d0, 1, depthNormalized.data(), depth.d1);

}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void serialize_csv(const std::map<std::string, arr>& dictionary, const std::string& filename) {
    create_dir(filename);
    std::ofstream ofs(filename);

    // Write each key-value pair
    for (const auto& pair : dictionary) {
        ofs << pair.first << "," << pair.second << "\n";
    }
    ofs.close();
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void serialize_csv(const std::vector<std::map<std::string, std::string>>& conf, const std::string& filename) {
    create_dir(filename);
    std::ofstream ofs(filename);

    for(const auto& dictionary : conf){
        for (const auto& pair : dictionary) {
            ofs << pair.first << "," << pair.second << "\n";
        }
    }
    ofs.close();
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void serialize_csv(const arr& array, const std::string& filename) {
    create_dir(filename);
    std::ofstream file(filename);

    // Check if the file is open
    if (!file) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return;
    }

    int rows = array.d0;
    int cols = array.d1;
    
    if(array.d2 == 0){
        // Write the array data to the file
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                file << array(i,j); // Convert uint8_t to int for proper CSV output
                file << ","; // Add comma between elements
            }
            file << "\n"; // End of row
        }
    } else {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                for (int k = 0; k < array.d2; k++) {
                    file << array(i,j,k); // Convert uint8_t to int for proper CSV output
                    file << ","; // Add comma between elements
                }
                file << "\n"; // End of row
            }
        }
    }

    file.close();
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::string to_string(const arr& array){
    std::string str = "";
    for (uint i = 0; i < array.d0; i++) {
        str += std::to_string(array(i));
        str += ",";
    }
    str += "\n"; 
    return str;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
std::string to_string(const rai::Shape& shape){
    if(shape._type == rai::ST_box) {
        return "box";
    } else if(shape._type == rai::ST_cylinder) {
        return "cylinder";
    } else if(shape._type == rai::ST_sphere) {
        return "sphere";
    } else if(shape._type == rai::ST_ssBox) {
        return "ssBox";
    } else if(shape._type == rai::ST_marker) {
        return "marker";
    } else {
        return "unknown";
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void serialize_C(const rai::Configuration& C, const std::string& filename) {
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << C << std::endl;
        outFile.close();
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void create_dir(const std::string& filepath) {
    std::string directory = filepath.substr(0, filepath.find_last_of('/'));
    mkdir(directory.c_str(), 0755); 
 }