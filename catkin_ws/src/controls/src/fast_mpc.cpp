#include <ros/ros.h>
#include <humanoid_msgs/ServoCommand.h>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <string>
#include <mutex>
#include <humanoid_msgs/ServoFeedback.h>
#include <unordered_map>
#include <cmath>
#include <boost/asio.hpp>

std::vector<std::string> JOINTS = {
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee"
};

std::vector<float> joint_pos = std::vector<float>(JOINTS.size(), 0.0);
std::vector<float> joint_vel = std::vector<float>(JOINTS.size(), 0.0);
std::vector<float> ang_vel = {0.0, 0.0, 0.0};
geometry_msgs::Quaternion quat;
std::mutex state_mutex;

boost::asio::io_context io_context;
boost::asio::ip::tcp::socket tcp_socket(io_context);
boost::asio::ip::tcp::resolver resolver(io_context);
boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve("localhost", "5555");


// Callback functions to update robot states
void jointStateCallback(const humanoid_msgs::ServoFeedback::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex);
    joint_pos.clear();
    joint_vel.clear();

    for (const auto& joint_name : JOINTS) {
        try {
            const std::vector<float>* joint_data = nullptr;
            
            if (joint_name == "right_shoulder_pitch") joint_data = &msg->right_shoulder_pitch;
            else if (joint_name == "right_shoulder_roll") joint_data = &msg->right_shoulder_roll;
            else if (joint_name == "right_elbow") joint_data = &msg->right_elbow;
            else if (joint_name == "left_shoulder_pitch") joint_data = &msg->left_shoulder_pitch;
            else if (joint_name == "left_shoulder_roll") joint_data = &msg->left_shoulder_roll;
            else if (joint_name == "left_elbow") joint_data = &msg->left_elbow;
            else if (joint_name == "left_hip_roll") joint_data = &msg->left_hip_roll;
            else if (joint_name == "left_hip_pitch") joint_data = &msg->left_hip_pitch;
            else if (joint_name == "left_knee") joint_data = &msg->left_knee;
            else if (joint_name == "right_hip_roll") joint_data = &msg->right_hip_roll;
            else if (joint_name == "right_hip_pitch") joint_data = &msg->right_hip_pitch;
            else if (joint_name == "right_knee") joint_data = &msg->right_knee;
            else throw std::runtime_error("Unknown joint name: " + joint_name);

            if ((*joint_data).size() >= 2) {
                float position = (*joint_data)[0];
                float velocity = (*joint_data)[1];

                // normalize joint position
                position = std::fmod(position, 2 * M_PI);
                if (position > M_PI) {
                    position -= 2 * M_PI;
                }

                joint_pos.push_back(position);
                joint_vel.push_back(velocity);
            } else {
                throw std::runtime_error("Insufficient data in joint array");
            }
        } catch (const std::out_of_range& e) {
            ROS_WARN_STREAM("Missing feedback for joint: " << joint_name);
            joint_pos.push_back(0.0f);
            joint_vel.push_back(0.0f);
        }
    }
}

void angularVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex);
    ang_vel = {static_cast<float>(msg->x), static_cast<float>(msg->y), static_cast<float>(msg->z)};
}

void quaternionCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex);
    quat = *msg;
}

void setupConnection() {
    try {
        boost::asio::connect(tcp_socket, endpoints);
        tcp_socket.non_blocking(true);
        std::cout << "Connected!" << std::endl;
    } catch (const boost::system::system_error& e) {
        ros::Duration(1.0).sleep();
    }
}

void generateControl(ros::Publisher& command_pub) {
    Json::Value state_json;
    {
        state_json["joint_pos"] = Json::arrayValue;
        state_json["joint_vel"] = Json::arrayValue;
        state_json["ang_vel"] = Json::arrayValue;
        state_json["quat"] = Json::arrayValue;
        std::lock_guard<std::mutex> lock(state_mutex);
        for (const auto& pos : joint_pos) state_json["joint_pos"].append(pos);
        for (const auto& vel : joint_vel) state_json["joint_vel"].append(vel);
        for (const auto& ang : ang_vel) state_json["ang_vel"].append(ang);
        for (const auto& q : {quat.w, quat.x, quat.y, quat.z}) state_json["quat"].append(q);
    }

    Json::StreamWriterBuilder writer;
    std::string str_state_json = Json::writeString(writer, state_json);

    try {
        boost::asio::write(tcp_socket, boost::asio::buffer(str_state_json));
        
        char reply[1024];
        size_t reply_length = tcp_socket.read_some(boost::asio::buffer(reply));
        std::string response_str(reply, reply_length);

        Json::Reader reader;
        Json::Value command_arr;
        reader.parse(response_str, command_arr);

        humanoid_msgs::ServoCommand command;
        for (size_t i = 0; i < JOINTS.size(); ++i) {
            
            Json::Value::ArrayIndex c_i = i; 

            if (JOINTS[i] == "right_shoulder_pitch") command.right_shoulder_pitch = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "right_shoulder_roll") command.right_shoulder_roll = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "right_elbow") command.right_elbow = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "left_shoulder_pitch") command.left_shoulder_pitch = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "left_shoulder_roll") command.left_shoulder_roll = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "left_elbow") command.left_elbow = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "left_hip_roll") command.left_hip_roll = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "left_hip_pitch") command.left_hip_pitch = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "left_knee") command.left_knee = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "right_hip_roll") command.right_hip_roll = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "right_hip_pitch") command.right_hip_pitch = command_arr[c_i].asFloat();
            else if (JOINTS[i] == "right_knee") command.right_knee = command_arr[c_i].asFloat();
            else throw std::runtime_error("Unknown joint name: " + JOINTS[i]);
        }
        command_pub.publish(command);
    } catch (const std::exception& e) {
        ROS_ERROR("Timeout while waiting for MPC server. Reconnecting...");
        setupConnection();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fast_mpc");
    ros::NodeHandle nh;

    ros::Publisher command_pub = nh.advertise<humanoid_msgs::ServoCommand>("/servos/command", 1);
    ros::Subscriber joint_sub = nh.subscribe("/servos/feedback", 1, jointStateCallback);
    ros::Subscriber ang_vel_sub = nh.subscribe("/state/ang_vel", 1, angularVelocityCallback);
    ros::Subscriber quat_sub = nh.subscribe("/state/quat", 1, quaternionCallback);

    double control_interval;
    nh.getParam("/controls/mpc/control_interval", control_interval);
    setupConnection();
    if (control_interval <= 0) {
        while (ros::ok()) {
            generateControl(command_pub);
        }
    } else {
        ros::Rate rate(control_interval > 0 ? 1.0 / control_interval : 100.0);
        while (ros::ok()) {
            generateControl(command_pub);
            rate.sleep();
        }
    }

    return 0;
}
