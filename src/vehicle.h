#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <vector>
#include <tuple>
#include "json.hpp"
#include "helpers.h"
#include "spline.h"

const double SPEED_LIMIT = 22.0; // 50mph = 22.352m/s
const double ACC_LIMIT = 9.0; // m/s^2  margin = 1.0m/s^2
const double JERK_LIMIT = 9.0; // m/s^3  margin = 1.0m/s^3
const int NUM_TRAJECTORY = 50;
const double DT = 0.02; // Delta time
const int NUM_LANE = 3;

enum class State { KL, PLCL, LCL, PLCR, LCR };

class Vehicle {
  public:
    Vehicle();

    static int get_lane(double d);
    void update(double x, double y, double s, double d, double yaw, double v);
    void next_state(nlohmann::basic_json<>& sensor_fusion);
    void print();
    std::tuple<std::vector<double>, std::vector<double>> get_tragectory(
        nlohmann::basic_json<>& previous_path_x,
        nlohmann::basic_json<>& previous_path_y,
        std::vector<double>& map_waypoints_x,
        std::vector<double>& map_waypoints_y,
        std::vector<double>& map_waypoints_s);
    
  private:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double vel;
    double target_vel;
    double order_vel;
    double acc;
    double target_acc;
    double order_acc;
    State state;
    std::vector<std::vector<int>> closest_vehicle_ids;

    int lane;
    int target_lane;

    std::vector<std::vector<int>> interpret_sensor_fusion(nlohmann::basic_json<>& sensor_fusion);
    double cost_keep_lane(nlohmann::basic_json<>& sensor_fusion);
    double cost_lane_change(int lane, nlohmann::basic_json<>& sensor_fusion);
    double cost_lane(int lane, nlohmann::basic_json<>& sensor_fusion);
    void keep_lane();
    void change_lane(int lane);
    void slow_down(nlohmann::basic_json<>& sensor_fusion);
    std::vector<std::vector<double>> get_wps_keep_lane(
        std::vector<double>& map_waypoints_x,
        std::vector<double>& map_waypoints_y,
        std::vector<double>& map_waypoints_s);
    std::vector<std::vector<double>> get_wps_lane_change(
        int target_lane,
        std::vector<double>& map_waypoints_x,
        std::vector<double>& map_waypoints_y,
        std::vector<double>& map_waypoints_s);

};

#endif
