#include "HMAPBiman.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

struct Quaternion {
    double w, x, y, z;
};
rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat = {0.7, 0, 0, 0.7});
void randomizeBox(rai::Configuration& C, rai::Configuration& C2);
void randomizeShelf(rai::Configuration& C, rai::Configuration& C2, StringA& all_frames, arr& qF);
double degreesToRadians(double degrees);
Quaternion eulerToQuaternion(double roll, double pitch, double yaw);
bool RRT(rai::Configuration& C2, const arr& qF, arr& path, bool view = true, double rrt_extend_length = 0.04);
void getImage(rai::Configuration& C, const std::string& cam_frame, byteA& img, floatA& depth, arr& pts);
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
    C.addFile("HMAP_bookshelf_conf.g");  
    C_copy.addFile("HMAP_bookshelf_conf.g"); 

    rai::Configuration C2;
    rai::Configuration C2_copy;
    C2.addFile("HMAP_bookshelf_actuated_conf.g");  
    C2_copy.addFile("HMAP_bookshelf_actuated_conf.g");

    C.view(true, "Initial Configuration");

    StringA all_frames = C.getFrameNames();
    uint count = 0;
    uint sample_count = 500;
    std::map<std::string, arr> dictionary_cam;
    std::string target = "box";
    std::vector<std::string> tool_list = {"stick"};
    std::vector<std::string> gripper_list = {"r_l_gripper", "l_l_gripper"};
    double filter = 0.01;
    std::string video_path = "video/config";
    int total_obstacle_count = 1;

    // Get camera info
    for (uint i = 0; i < 3; i++){
        rai::Frame* cam_frame = C.getFrame(("cam_frame_" + std::to_string(i)).c_str());
        dictionary_cam["camera" + std::to_string(i) + "_position"] = cam_frame->getPosition();
        dictionary_cam["camera" + std::to_string(i) + "_rotation"] = cam_frame->getRotationMatrix();
    }

    for(uint i = 0; i < sample_count; i++) {
        std::string base_name = "sample/bookshelf" + std::to_string(count) + "/";
        arr qF = {0.20, -0.57, 1.29, -1, 0, 0, 0};
        arr q_obs = {-0.19, -0.4, 0.95, 1, 0, 0, 0};
        C = C_copy;
        C2 = C2_copy;
        arr path;
        arr contact_points;
        addMarker(C, {qF(0), qF(1), qF(2)}, "cp", "shelf", 0.2, false, {qF(3), qF(4), qF(5), qF(6)});
        addMarker(C, {q_obs(0), q_obs(1), q_obs(2)}, "q_obs", "shelf", 0.2, false, {q_obs(3), q_obs(4), q_obs(5), q_obs(6)});

        // Randomize box
        randomizeBox(C, C2);

        // Randomize the shelf
        randomizeShelf(C, C2, all_frames, qF);
        
        C2.setJointState(C.getFrame("box")->getPose());
        qF = C.getFrame("cp")->getPosition().append({qF(3), qF(4), qF(5), qF(6)});
        q_obs = C.getFrame("q_obs")->getPosition().append({q_obs(3), q_obs(4), q_obs(5), q_obs(6)});
        C2.view(true, "Randomized Configuration");

        // Generate waypoints
        HMAPBiman hmap_biman(C, C2, qF, q_obs, target, total_obstacle_count, tool_list, gripper_list, filter, video_path, true);
        if(hmap_biman.run()){
            serialize_csv(dictionary_cam, base_name + "cam_info.csv");
            serialize_csv(path, base_name + "waypoint_"+std::to_string(count) + ".csv");
            contact_points.reshape(contact_points.d0/3, 3);
            serialize_csv(contact_points, base_name + "contact_point_"+std::to_string(count) + ".csv");

            for(uint j = 0; j < 3; j++){
                byteA img;
                floatA depth;
                arr pts;
                getImage(C, "cam_frame_"+std::to_string(j), img, depth, pts);
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
    arr xyz_box = box->getPosition();
    arr rpy_box = {0, 0, rnd.uni(-10, 10)};
    arr size_box = box->getSize();
    Quaternion q_box = eulerToQuaternion(rpy_box(0), rpy_box(1), rpy_box(2));
    double scale_box = rnd.uni(0.7, 1);
    arr pos_dif = {rnd.uni(-0.08, 0.08), 0, 0};
    xyz_box += pos_dif;
    size_box *= {scale_box, scale_box, 1, 1};
    box->setQuaternion({q_box.w, q_box.x, q_box.y, q_box.z});    
    box->setPosition(xyz_box);
    box->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
    box->setShape(rai::ST_box, size_box);
    //box2->setQuaternion({q_box.w, q_box.x, q_box.y, q_box.z});   
    //box2->setPosition(xyz_box);
    //box2->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
    //box2->setShape(rai::ST_box, size_box);
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void randomizeShelf(rai::Configuration& C, rai::Configuration& C2, StringA& all_frames, arr& qF){
    rai::Frame* shelf = C.getFrame("shelf");
    rai::Frame* shelf2 = C2.getFrame("shelf");
    arr rpy_shelf = {0, 0, rnd.uni(-6, 6)};
    arr xyz_shelf = {rnd.uni(-0.05, 0.05), rnd.uni(-0.1, 0.1), 0};
    Quaternion q_shelf = eulerToQuaternion(rpy_shelf(0), rpy_shelf(1), rpy_shelf(2));
    shelf->setQuaternion({q_shelf.w, q_shelf.x, q_shelf.y, q_shelf.z});    
    shelf->setPosition(xyz_shelf);
    shelf2->setQuaternion({q_shelf.w, q_shelf.x, q_shelf.y, q_shelf.z});    
    shelf2->setPosition(xyz_shelf);
    double scale_shelf = rnd.uni(0.9, 1.1);
    qF += {xyz_shelf(0), xyz_shelf(1), xyz_shelf(2), 1+q_shelf.w, q_shelf.x, q_shelf.y, q_shelf.z};
    for (const auto& frame : all_frames){
        if (frame.contains("shelf_")) {
            rai::Frame* obj = C.getFrame(frame);
            rai::Frame* obj2 = C2.getFrame(frame);
            obj->setColor({rnd.uni(), rnd.uni(), rnd.uni()});
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
void getImage(rai::Configuration& C, const std::string& cam_frame, byteA& img, floatA& depth, arr& pts){
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
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
Quaternion multiplyQuaternions(const Quaternion& q1, const Quaternion& q2) {
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
Quaternion addEulerToQuaternion(const Quaternion& q, double roll, double pitch, double yaw) {
    // Convert the Euler angles to a quaternion
    Quaternion eulerQuat = eulerToQuaternion(roll, pitch, yaw);

    // Multiply the original quaternion by the Euler quaternion to apply the rotation
    return multiplyQuaternions(q, eulerQuat);
}
