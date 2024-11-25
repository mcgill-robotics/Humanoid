#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <humanoid_msgs/HardwareServoCommand.h>
#include <humanoid_msgs/ServoCommand.h>
#include <humanoid_msgs/ServoFeedback.h>
#include <humanoid_msgs/HardwareServoFeedback.h>
#include <std_srvs/Trigger.h>
#include <cmath>
#include <map>
#include <string>
#include <vector>

std::map<std::string, std::string> JOINTS_MAPPINGS_SOFTWARE_HARDWARE = {
    {"right_shoulder_pitch", "right_shoulder_pitch"},
    {"right_shoulder_roll", "right_shoulder_roll"},
    {"right_elbow", "right_elbow"},
    {"left_shoulder_pitch", "left_shoulder_pitch"},
    {"left_shoulder_roll", "left_shoulder_roll"},
    {"left_elbow", "left_elbow"},
    {"left_hip_roll", "left_hip_roll"},
    {"left_hip_pitch", "left_hip_pitch"},
    {"left_knee", "left_knee"},
    {"right_hip_roll", "right_hip_roll"},
    {"right_hip_pitch", "right_hip_pitch"},
    {"right_knee", "right_knee"}
};

bool allow_commands = false;
int control_mode = 0;
ros::Publisher setpoint_publisher;
ros::Publisher feedback_publisher;
ros::Publisher control_mode_change_pub;

double modulo_angle(double angle) {
    return fmod(angle + 180.0, 360.0) - 180.0;
}

double radians_to_degrees(double radians_value) {
    return modulo_angle(radians_value * (180.0 / M_PI));
}

double degrees_to_radians(double degrees_value) {
    return modulo_angle(degrees_value) / (180.0 / M_PI);
}

double clamp(double x, double lower_limit, double upper_limit) {
    return std::max(lower_limit, std::min(x, upper_limit));
}

void _position_command_cb(const humanoid_msgs::ServoCommand::ConstPtr& msg) {
    if (!allow_commands) return;

    humanoid_msgs::HardwareServoCommand setpoints;

    for (const auto& pair : JOINTS_MAPPINGS_SOFTWARE_HARDWARE) {
        const std::string& software_joint = pair.first;
        const std::string& hardware_joint = pair.second;

        XmlRpc::XmlRpcValue joint_specs;
        ros::param::get("~" + software_joint, joint_specs);

        int direction = static_cast<int>(joint_specs[2]);
        double joint_center;
        ros::param::get("~" + software_joint + "_center", joint_center);

        double upper_joint_limit, lower_joint_limit;
        if (direction > 0) {
            upper_joint_limit = static_cast<double>(joint_specs[0]);
            lower_joint_limit = static_cast<double>(joint_specs[1]);
        } else if (direction < 0) {
            upper_joint_limit = -static_cast<double>(joint_specs[1]);
            lower_joint_limit = -static_cast<double>(joint_specs[0]);
        }

        double joint_setpoint = joint_center + clamp(
            direction * radians_to_degrees(msg->joint_positions[software_joint]),
            lower_joint_limit, upper_joint_limit);

        setpoints.joint_positions[hardware_joint] = joint_setpoint;
    }

    setpoint_publisher.publish(setpoints);
}

void _torque_command_cb(const humanoid_msgs::ServoCommand::ConstPtr& msg) {
    if (!allow_commands) return;

    humanoid_msgs::HardwareServoCommand setpoints;

    for (const auto& pair : JOINTS_MAPPINGS_SOFTWARE_HARDWARE) {
        const std::string& software_joint = pair.first;
        const std::string& hardware_joint = pair.second;

        XmlRpc::XmlRpcValue joint_specs;
        ros::param::get("~" + software_joint, joint_specs);

        int direction = static_cast<int>(joint_specs[2]);

        double torque_setpoint = 885 * (direction * msg->joint_positions[software_joint] / 1.4);
        setpoints.joint_positions[hardware_joint] = torque_setpoint;
    }

    setpoint_publisher.publish(setpoints);
}

void feedbackCb(const humanoid_msgs::HardwareServoFeedback::ConstPtr& msg) {
    humanoid_msgs::ServoFeedback feedback;

    for (const auto& pair : JOINTS_MAPPINGS_SOFTWARE_HARDWARE) {
        const std::string& software_joint = pair.first;
        const std::string& hardware_joint = pair.second;

        XmlRpc::XmlRpcValue joint_specs;
        ros::param::get("~" + software_joint, joint_specs);

        int direction = static_cast<int>(joint_specs[2]);

        double joint_center;
        ros::param::get("~" + software_joint + "_center", joint_center);

        std::vector<double> hw_feedback_tuple = msg->joint_positions[hardware_joint];
        std::vector<double> sw_feedback_tuple;

        if (!hw_feedback_tuple.empty()) {
            double joint_pos = direction * degrees_to_radians(hw_feedback_tuple[0] - joint_center);
            sw_feedback_tuple.push_back(joint_pos);
        }

        if (hw_feedback_tuple.size() > 1) {
            double joint_vel = direction * degrees_to_radians(hw_feedback_tuple[1]);
            sw_feedback_tuple.push_back(joint_vel);
        }

        if (hw_feedback_tuple.size() > 2) {
            double joint_load = direction * hw_feedback_tuple[2];
            sw_feedback_tuple.push_back(joint_load);
        }

        feedback.joint_positions[software_joint] = sw_feedback_tuple;
    }

    feedback_publisher.publish(feedback);
}

void control_mode_cb(const std_msgs::Float32::ConstPtr& msg) {
    allow_commands = false;

    if (msg->data == 0) {
        control_mode_change_pub.publish(msg);
        control_mode = 0;
    } else if (msg->data == 1) {
        control_mode_change_pub.publish(msg);
        control_mode = 1;
    } else {
        ROS_WARN("Invalid control mode: %f. Continuing with previous control mode.", msg->data);
    }

    ros::Duration(1.0).sleep();
    allow_commands = true;
}

void commandCb_caller(const humanoid_msgs::ServoCommand::ConstPtr& msg) {
    if (control_mode == 0) {
        _position_command_cb(msg);
    } else if (control_mode == 1) {
        _torque_command_cb(msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hardware_interface");

    ros::NodeHandle nh;

    ros::Subscriber control_mode_sub = nh.subscribe("/servos/set_control_mode", 1, control_mode_cb);
    control_mode_change_pub = nh.advertise<std_msgs::Float32>("hardware/servos/set_control_mode", 1);

    ros::Subscriber command_sub = nh.subscribe("/servos/command", 1, commandCb_caller);
    setpoint_publisher = nh.advertise<humanoid_msgs::HardwareServoCommand>("/hardware/servos/command", 1);

    ros::Subscriber feedback_sub = nh.subscribe("/hardware/servos/feedback", 1, feedbackCb);
    feedback_publisher = nh.advertise<humanoid_msgs::ServoFeedback>("/servos/feedback", 1);

    ros::spin();

    return 0;
}
