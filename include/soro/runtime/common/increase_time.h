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
using namespace soro;
enum phase_type { acceleration, braking, cruising, invalid };

using phases = vector<vector<runtime::train_state>>;
using types = vector<phase_type>;

struct train_drive {
  static constexpr train_drive empty(){return {{},{}};}
  void merge_phases(int const& offset);
  void push_back(vector<runtime::train_state>const& phase,phase_type const& type);
  void insert(int const& offset,phases const& new_phases,vector<phase_type> const& types);
  void fix_times(int const& offset);
  void fix_phases(int const& offset);
  void fix_drive(int const& offset);
  void erase_elements(int const& offset,
                      int const& to_delete);
  train_drive& operator+=(train_drive const& other);
  void print();

  phases phases_;
  types phase_types_;
  soro::runtime::train_state start_state_;
};

si::time get_cruise_time(si::speed const& speed, si::length const& start,
                         si::length const& stop);
si::time get_cruise_time(runtime::train_state const& start, runtime::train_state const& end);

void set_pt(train_path_envelope::tpe_point const& point);

void set_intervals(vector<runtime::interval_point> const& intr_points);

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

bool check_HA(train_drive& drive,rs::train_physics const& tp);

//bool check_DH(train_drive& drive);

bool check_DA(train_drive& drive,rs::train_physics const& tp);

/*bool check_A(train_drive& drive);

bool check_H(train_drive& drive);

bool check_D(train_drive& drive);*/

bool slowest_drive(train_drive& drive,int const& cruise_index,rs::train_physics const& tp);
}// namespace increase_time