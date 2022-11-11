//
// Created by roboticslab on 11/11/22.
//

#ifndef CREATE3_CONTROLLER_PARAM_MANAGER_H
#define CREATE3_CONTROLLER_PARAM_MANAGER_H

#include <yaml-cpp/yaml.h>
#include <memory>
#include <iostream>
#define DEBUG(x) std::cout << x << std::endl
using namespace std;
class param_manager;

typedef shared_ptr<param_manager> ParamPtr;
struct PIDGains{
    double kp, kd, ki;
};

struct PIDStability{
    double dt, deadzone, timeout;
};

class param_manager: enable_shared_from_this<param_manager>{
public:
    param_manager(const string& file):param_file_(file)
    {
        DEBUG("loading " << file);
        YAML::Node config = YAML::LoadFile(file);
        parse_file(config);
    }

    ParamPtr get_ptr()
    {
        return shared_from_this();
    }

    PIDGains get_position_gains()
    {
        return positionGains_;
    }

    PIDStability get_position_stability()
    {
        return positionStability_;
    }

    PIDGains get_orientation_gains()
    {
        return orientationGains_;
    }

    PIDStability get_orientation_stability()
    {
        return orientationStatbility_;
    }

    string get_path_file()
    {
        return path_file_;
    }
protected:
    void parse_file(YAML::Node& config)
    {
        positionGains_.kp = config["controller"]["position"]["gains"]["kp"].as<double>();
        positionGains_.kd = config["controller"]["position"]["gains"]["kd"].as<double>();
        positionGains_.ki = config["controller"]["position"]["gains"]["ki"].as<double>();

        positionStability_.dt        = config["controller"]["position"]["stability"]["dt"].as<double>();
        positionStability_.deadzone  = config["controller"]["position"]["stability"]["deadzone"].as<double>();
        positionStability_.timeout   = config["controller"]["position"]["stability"]["timeout"].as<double>();

        orientationGains_.kp = config["controller"]["orientation"]["gains"]["kp"].as<double>();
        orientationGains_.kd = config["controller"]["orientation"]["gains"]["kd"].as<double>();
        orientationGains_.ki = config["controller"]["orientation"]["gains"]["ki"].as<double>();

        orientationStatbility_.dt       = config["controller"]["orientation"]["stability"]["dt"].as<double>();
        orientationStatbility_.deadzone = config["controller"]["orientation"]["stability"]["deadzone"].as<double>();
        orientationStatbility_.timeout  = config["controller"]["orientation"]["stability"]["timeout"].as<double>();


        path_file_ = config["path_file"].as<string>();
    }


private:
    const string param_file_;

    std::string path_file_;
    PIDGains positionGains_, orientationGains_;
    PIDStability positionStability_, orientationStatbility_;

};
#endif //CREATE3_CONTROLLER_PARAM_MANAGER_H
