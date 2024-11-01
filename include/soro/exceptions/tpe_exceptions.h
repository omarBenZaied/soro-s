#pragma once
namespace soro::tpe_simulation{
  struct tpe_l_time_exception : std::runtime_error{
    explicit tpe_l_time_exception(const std::string& what_arg,tpe_point const& tpe_point,runtime::train_state const& state,vector<runtime::interval_point> const& intr_points)
        : std::runtime_error(what_arg),state_(state),point_(tpe_point),intr_points_(intr_points) {}

    runtime::train_state const state_;
    tpe_point const point_;
    vector<runtime::interval_point> intr_points_;
  };
  struct tpe_speed_exception : std::runtime_error{
    explicit tpe_speed_exception(const std::string& what_arg,tpe_point const& tpe_point,runtime::train_state const& state,vector<runtime::interval_point> const& intr_points)
      : std::runtime_error(what_arg),state_(state),point_(tpe_point),intr_points_(intr_points) {}
    runtime::train_state const state_;
    tpe_point const point_;
    vector<runtime::interval_point> intr_points_;
  };
} // namspace soro::tpe_simulation