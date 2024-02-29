///
/// \file ekf_slam.cpp
/// \author your Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief The libary for all slam calculations
/// \version 0.1
/// \date 2024-02-25
///
/// \copyright Copyright (c) 2024
///
///
#include <armadillo>
#include <limits>

#include "turtlelib/ekf_slam.hpp"

namespace turtlelib
{
EKF::EKF()
{
  EKF(20);
}

EKF::EKF(int num_obstacles)
{
  num_obstacles_ = num_obstacles;
  state_ = {0.0, 0.0, 0.0};

  arma::mat up_left(3, 3, arma::fill::zeros);
  arma::mat up_right(3, 2 * num_obstacles, arma::fill::zeros);
  arma::mat bottom_left(2 * num_obstacles, 3, arma::fill::zeros);
  arma::mat bottom_right = 1e10 * arma::mat(
    2 * num_obstacles,
    2 * num_obstacles,
    arma::fill::eye);

  covariance_mat_ = arma::join_horiz(
    arma::join_vert(up_left, bottom_left),
    arma::join_vert(up_right, bottom_right));

  for (int i = 0; i < num_obstacles; ++i) {
    obstacles_.push_back({1e4, 1e4, -1});
  }
}

arma::vec EKF::get_state_vec()
{
  arma::vec state(3 + 2 * num_obstacles_);

  state.at(0) = state_.theta;
  state.at(1) = state_.x;
  state.at(2) = state_.y;

  for (size_t i = 0; i < num_obstacles_; ++i) {
    const auto uid = obstacles_.at(i).uid;
    const auto x = obstacles_.at(i).x;
    const auto y = obstacles_.at(i).y;

    if (uid != -1) {
      state(3 + 2 * uid) = x;
      state(3 + 2 * uid + 1) = y;
    } else {
      state(3 + 2 * i) = x;
      state(3 + 2 * i + 1) = y;
    }
  }

  return state;
}

std::vector<Measurement> EKF::get_all_landmarks()
{
  return obstacles_;
}

Measurement EKF::get_landmark_pos(int uid)
{
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    Measurement obs = obstacles_.at(i);

    if (obs.uid == uid) {
      return obs;
    }
  }

  return {1e4, 1e4, -1};
}

arma::vec EKF::get_h_vec(Measurement landmark)
{
  const auto dx = landmark.x;
  const auto dy = landmark.y;

  return {sqrt(pow(dx, 2.0) + pow(dy, 2.0)), atan2(dy, dx)};
}

arma::mat EKF::get_H_mat(Measurement landmark, int index)
{
  const auto dx = landmark.x;
  const auto dy = landmark.y;
  const auto d = pow(dx, 2.0) + pow(dy, 2.0);

  const arma::mat first = {
    {0, -dx / sqrt(d), -dy / sqrt(d)},
    {-1.0, dy / d, -dx / d}
  };

  // arma::mat second;
  // if (index != 0) {
  //   second = arma::mat(2, 2 * index, arma::fill::zeros);
  // }
  const arma::mat second(2, 2 * index, arma::fill::zeros);

  const arma::mat third = {
    {dx / sqrt(d), dy / sqrt(d)},
    {-dy / d, dx / d}
  };

  const arma::mat last(2, 2 * num_obstacles_ - 2 * index - 2, arma::fill::zeros);

  if (index == 0) {
    return arma::join_horiz(
      arma::join_horiz(first, third),
      last
    );
  } else {
    return arma::join_horiz(
      arma::join_horiz(first, second),
      arma::join_horiz(third, last)
    );
  }
}

arma::mat EKF::get_A_mat(double dx, double dy)
{
  // const auto dx = body_twist.x;
  // const auto dtheta = body_twist.omega;

  arma::mat I_mat(2 * num_obstacles_ + 3, 2 * num_obstacles_ + 3, arma::fill::eye);

  const arma::mat up_right(3, 2 * num_obstacles_, arma::fill::zeros);
  const arma::mat down_left(2 * num_obstacles_, 3, arma::fill::zeros);
  const arma::mat down_right(2 * num_obstacles_, 2 * num_obstacles_, arma::fill::zeros);

  arma::mat up_left = {
    {0, 0, 0},
    {-dy, 0, 0},
    {dx, 0, 0}
  };

  // if (almost_equal(dtheta, 0.0)) {
  //   up_left = {{0, 0, 0},
  //     {-dx * sin(state_.theta), 0, 0},
  //     {dx * cos(state_.theta), 0, 0}};

  // } else {
  //   up_left = {{0, 0, 0},
  //     {-dx / dtheta * cos(state_.theta) + dx / dtheta * cos(state_.theta + dtheta), 0, 0},
  //     {-dx / dtheta * sin(state_.theta) + dx / dtheta * sin(state_.theta + dtheta), 0, 0}};
  // }

  const arma::mat A_mat = I_mat +
    arma::join_horiz(
    arma::join_vert(up_left, down_left),
    arma::join_vert(up_right, down_right));

  return A_mat;
}

void EKF::update_covariance(arma::mat sigma_new)
{
  covariance_mat_ = sigma_new;
}

void EKF::update_state(double x, double y, double theta)
{
  state_.x = x;
  state_.y = y;
  state_.theta = theta;
}

void EKF::update_landmark_pos(arma::vec state)
{
  // for (size_t i = 0; i < landmarks.size(); ++i) {
  //   const auto x = landmarks.at(i).x;
  //   const auto y = landmarks.at(i).y;
  //   const auto uid = landmarks.at(i).uid;

  //   if (!almost_equal(x, 100.0) && !almost_equal(y, 100.0)) {
  //     Point2D pb{x, y};
  //     Transform2D Tsb{{state_.x, state_.y}, state_.theta};
  //     Point2D ps = Tsb(pb);

  //     obstacles_.at(uid).x = ps.x;
  //     obstacles_.at(uid).y = ps.y;
  //     obstacles_.at(uid).uid = uid;
  //   }
  for (int i = 0; i < num_obstacles_; ++i) {
    const auto x = state(3 + 2 * i);
    const auto y = state(3 + 2 * i + 1);

    if (!almost_equal(x, 1e4) && !almost_equal(y, 1e4)) {
      obstacles_.at(i).x = x;
      obstacles_.at(i).y = y;
      obstacles_.at(i).uid = i;
    }
  }
  // }

  // if (!landmark_pos_ready_) {
  //   landmark_pos_ready_ = true;
  // }
}

// bool EKF::is_landmark_pos_ready() const
// {
//   return landmark_pos_ready_;
// }

RobotState EKF::get_robot_state() const
{
  return state_;
}

arma::mat EKF::get_covariance_mat() const
{
  return covariance_mat_;
}
} // namespace turtlelib
