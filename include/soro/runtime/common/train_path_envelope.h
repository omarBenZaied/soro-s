//
// Created by Omar Ben Zaied on 13.08.2024.
//
#include "soro/si/units.h"
#include "soro/base/soro_types.h"
#pragma once
namespace soro::train_path_envelope{
struct tpe_point{
  tpe_point(si::length const& start,si::time const& e_time, si::time const& l_time,si::speed const& v_min,si::speed const& v_max)
      :distance_{start},e_time_{e_time},l_time_{l_time},v_min_{v_min},v_max_{v_max}{}

  si::length distance_;
  si::time e_time_;
  si::time l_time_;
  si::speed v_min_;
  si::speed v_max_;

  bool operator<(tpe_point const o) const{
    return this->distance_<o.distance_;
  }
  tpe_point operator=(tpe_point o){
    return o;
  }
};
using tpe_points = soro::vector<tpe_point>;
} // namespace soro::train_path_envelope