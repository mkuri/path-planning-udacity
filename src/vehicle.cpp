// #include <iostream>
// #include "helpers.h"
#include "vehicle.h"
// #include "spline.h"

Vehicle::Vehicle() {};

int Vehicle::get_lane(double d) {
  int lane = d / 4;
  return lane;
}

void Vehicle::update(double x, double y, double s, double d, double yaw, double v) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->vel = v;

  this->lane = Vehicle::get_lane(this->d);

}

void Vehicle::print() {
  std::cout << "x: " << this->x << ", y: " << this->y << std::endl;
  std::cout << "s: " << this->s << ", d: " << this->d << std::endl;
  std::cout << "yaw: " << this->yaw << ", vel: " << this->vel << std::endl;
  std::cout << "lane: " << this->lane << std::endl;
  std::cout << "target_vel: " << this->target_vel << ", target_acc: " << this->target_acc << std::endl;
}


void Vehicle::next_state() {
  this->target_vel = SPEED_LIMIT;
  this->target_acc = ACC_LIMIT;
}

std::tuple<std::vector<double>, std::vector<double>> Vehicle::get_tragectory(
    nlohmann::basic_json<>& previous_path_x,
    nlohmann::basic_json<>& previous_path_y,
    std::vector<double>& map_waypoints_x,
    std::vector<double>& map_waypoints_y,
    std::vector<double>& map_waypoints_s) {

  std::vector<double> next_x_vals, next_y_vals, target_xs, target_ys;
  int prev_size = previous_path_x.size();
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = deg2rad(this->yaw);
  double ref_vel = this->vel;

  // Append previous points to target xy points
  std::cout << "prev_size = " << prev_size << std::endl;
  if (prev_size < 2) {
    double prev_x = this->x - cos(this->yaw);
    double prev_y = this->y - sin(this->yaw);

    target_xs.push_back(prev_x);
    target_xs.push_back(this->x);
    target_ys.push_back(prev_y);
    target_ys.push_back(this->y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    double ref_prev_x = previous_path_x[prev_size-2];
    double ref_prev_y = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);
    ref_vel = this->order_vel;

    target_xs.push_back(ref_prev_x);
    target_xs.push_back(ref_x);
    target_ys.push_back(ref_prev_y);
    target_ys.push_back(ref_y);
  }
  std::cout << "previous points x: " << target_xs[0] << target_xs[1] << std::endl;
  std::cout << "previous points y: " << target_ys[0] << target_ys[1] << std::endl;
  // Append previous points to the output points for continuity
  for (int i = 0; i < prev_size; ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  int target_lane = 1;
  // Generate target xy points
  std::vector<double> wp0 = getXY(this->s+30, 2+4*target_lane,
      map_waypoints_s, map_waypoints_x, map_waypoints_y);
  std::vector<double> wp1 = getXY(this->s+60, 2+4*target_lane,
      map_waypoints_s, map_waypoints_x, map_waypoints_y);
  std::vector<double> wp2 = getXY(this->s+90, 2+4*target_lane,
      map_waypoints_s, map_waypoints_x, map_waypoints_y);

  target_xs.push_back(wp0[0]);
  target_xs.push_back(wp1[0]);
  target_xs.push_back(wp2[0]);
  target_ys.push_back(wp0[1]);
  target_ys.push_back(wp1[1]);
  target_ys.push_back(wp2[1]);

  std::cout << "target_xs: " << std::endl;
  for(auto tx: target_xs) {
    std::cout << tx << ", ";
  }
  std::cout << std::endl;
  std::cout << "target_ys: " << std::endl;
  for(auto ty: target_ys) {
    std::cout << ty << ", ";
  }
  std::cout << std::endl;

  // Map to the spline space
  for (int i = 0; i < target_xs.size(); ++i) {
    double shift_x = target_xs[i] - ref_x;
    double shift_y = target_ys[i] - ref_y;

    target_xs[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    target_ys[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  std::cout << "target_xs mapped to the spline space: " << std::endl;
  for(auto tx: target_xs) {
    std::cout << tx << ", ";
  }
  std::cout << std::endl;
  std::cout << "target_ys mapped to the spline space: " << std::endl;
  for(auto ty: target_ys) {
    std::cout << ty << ", ";
  }
  std::cout << std::endl;

  // Interpolate using spline
  tk::spline s;
  s.set_points(target_xs, target_ys);
  double spline_target_x = 30.0;
  double spline_target_y = s(spline_target_x);
  double spline_target_dist = distance(
      0, 0, spline_target_x, spline_target_y);
  std::cout << "spline graph: " << std::endl;
  for (int i = 0; i < 50; ++i) {
    double x = i;
    y = s(x);
    std::cout << "(" << x << ", " << y << "), ";
  }
  std::cout << std::endl;

  std::cout << "target_xs mapped to the xy space: " << std::endl;
  double x_add_on = 0;
  std::cout << "ref_vel: ";
  for (int i = 1; i < NUM_TRAJECTORY - prev_size; ++i) {
    if (ref_vel < this->target_vel - this->target_acc * DT) {
      ref_vel += this->target_acc * DT;
    } else if (ref_vel < this->target_vel + this->target_acc * DT) {
      ref_vel -= this->target_acc * DT;
    }
    std::cout << ref_vel << ", ";
    double n = (spline_target_dist/(ref_vel*DT));
    double point_x = x_add_on + spline_target_x/n;
    double point_y = s(point_x);

    x_add_on = point_x;

    // Map to the xy space
    double shift_x = point_x;
    double shift_y = point_y;
    point_x = (shift_x*cos(ref_yaw) - shift_y*sin(ref_yaw));
    point_y = (shift_x*sin(ref_yaw) + shift_y*cos(ref_yaw));
    point_x += ref_x;
    point_y += ref_y;

    next_x_vals.push_back(point_x);
    next_y_vals.push_back(point_y);
  }
  this->order_vel = ref_vel;
  std::cout << std::endl;

  std::cout << "target_xs splined and mapped to the xy space: " << std::endl;
  for(auto x: next_x_vals) {
    std::cout << x << ", ";
  }
  std::cout << std::endl;
  std::cout << "target_ys splined and mapped to the xy space: " << std::endl;
  for(auto y: next_y_vals) {
    std::cout << y << ", ";
  }
  std::cout << std::endl;

  auto trajectory = std::make_tuple(next_x_vals, next_y_vals);
  return trajectory;
}
