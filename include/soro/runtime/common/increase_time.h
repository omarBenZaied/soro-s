//
// Created by omarb on 04.09.2024.
//
#include "soro/base/soro_types.h"
#include "soro/rolling_stock/train_physics.h"
#include "soro/runtime/common/train_state.h"
#include "train_path_envelope.h"
#include "interval.h"
#pragma once
namespace increase_time {
enum phase_type { acceleration, braking, cruising, invalid };

using phases = soro::vector<soro::vector<soro::runtime::train_state>>;
using types = soro::vector<phase_type>;

struct train_drive {
  phases phases_;
  types phase_types_;
  void merge_phases(int const& offset);
};
using namespace soro;

void increase_time(train_drive& drive,
                   train_path_envelope::tpe_point const& point,
                   rs::train_physics const& tp,
                   vector<runtime::interval_point> const& interval_points,
                   std::function<int(train_drive const&)> const& get_offset);

bool check_AHD(train_drive& drive, int const& offset);

bool check_AHA(train_drive& drive, rs::train_physics const& tp,
               int const& offset);

bool check_HDH(train_drive& drive, rs::train_physics const& tp,
               int const& offset);

bool check_DHA(train_drive& drive, rs::train_physics const& tp,
               int const& offset);

bool check_HA(train_drive& drive);

bool check_DH(train_drive& drive);

bool check_DA(train_drive& drive);

bool check_A(train_drive& drive);

bool check_H(train_drive& drive);

bool check_D(train_drive& drive);
}// namespace increase_time