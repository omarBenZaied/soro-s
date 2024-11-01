#include "soro/runtime/common/increase_time.h"
#include "soro/rolling_stock/train_physics.h"
#include "soro/runtime/common/interval.h"
#include "soro/runtime/common/phase_checkers.h"
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/physics/rk4/brake.h"
#include "soro/runtime/physics/rk4/detail/delta_t.h"
#include "soro/runtime/physics/rk4/detail/get_intersection.h"
#include "soro/runtime/physics/rk4/detail/rk4_step.h"
#pragma once
namespace increase_time {
using namespace soro;
using namespace soro::runtime;

train_path_envelope::tpe_point pt;
vector<interval_point> intr_points;

void set_pt(train_path_envelope::tpe_point const& point) {
  pt.distance_ = point.distance_;
  pt.e_time_ = point.e_time_;
  pt.l_time_ = point.l_time_;
  pt.v_min_ = point.v_min_;
  pt.v_max_ = point.v_max_;
}

void set_intervals(vector<interval_point> const& interval_points) {
  intr_points.erase(intr_points.begin(), intr_points.end());
  intr_points.insert(intr_points.begin(), interval_points.begin(),
                     interval_points.end());
}

si::time get_cruise_time(si::speed const& speed, si::length const& start,
                         si::length const& stop) {
  return (stop - start) / speed;
}
si::time get_cruise_time(train_state const& start, train_state const& end) {
  utls::sassert(start.speed_ == end.speed_,
                "Cruise time demanded for non cruise");
  return get_cruise_time(start.speed_, start.dist_, end.dist_);
}

void trim_drive(train_drive& drive) {
  for(int i=0;i<drive.phases_.size();++i) {
    auto& phase = drive.phases_[i];
    phase.erase(phase.begin()+1,phase.end()-1);
    phase.shrink_to_fit();
  }
  drive.phases_.shrink_to_fit();
}

void fix_times_cruise(vector<train_state>& phase, train_state const& before) {
  phase[0].time_ = before.time_;
  phase[1].time_ = phase[0].time_ + get_cruise_time(phase[0], phase[1]);
}
void fix_times_non_cruise(vector<train_state>& phase,
                          train_state const& before) {
  auto time_offset = before.time_ - phase[0].time_;
  for (int i = 0; i < phase.size(); ++i) {
    phase[i].time_ += time_offset;
  }
}
void train_drive::fix_times(int const& offset) {
  utls::sassert(phases_.size() == phase_types_.size(),
                "There are not as many types as phases");
  for (int i = offset; i < phase_types_.size(); ++i) {
    auto predecessor = i > 0 ? phases_[i - 1].back() : start_state_;
    switch (phase_types_[i]) {
      case cruising: fix_times_cruise(phases_[i], predecessor); break;
      case braking:
      case acceleration: fix_times_non_cruise(phases_[i], predecessor); break;
      default: utl::fail("Invalid phase type detected");
    }
  }
}

void train_drive::fix_phases(int const& start_offset) {
  auto i = start_offset;
  while (i < phase_types_.size() - 1) {
    if (phase_types_[i] == phase_types_[i + 1]) {
      merge_phases(i);
    } else
      ++i;
  }
}
void train_drive::fix_drive(int const& offset) {
  fix_times(offset);
  fix_phases(offset);
}

void train_drive::merge_phases(const int& offset) {
  utls::sassert(phase_types_[offset] == phase_types_[offset + 1],
                "the two phases to be merged arent equal");
  phase_types_.erase(phase_types_.begin() + offset + 1);
  switch (phase_types_[offset]) {
    case acceleration:
    case braking:
      phases_[offset].insert(phases_[offset].end(),
                             phases_[offset + 1].begin() + 1,
                             phases_[offset + 1].end());
      break;
    case cruising:
      phases_[offset].pop_back();
      phases_[offset].push_back(phases_[offset + 1].back());
      break;
    default: throw std::logic_error("Invalid phase type detected");
  }
  phases_.erase(phases_.begin() + offset + 1);
}

void train_drive::push_back(vector<runtime::train_state> const& phase,
                            phase_type const& type) {
  phases_.push_back(phase);
  phase_types_.push_back(type);
}

train_drive& train_drive::operator+=(const train_drive& other) {
  if (other.phases_.empty()) return *this;
  auto offset = phases_.size();
  train_state predecessor;
  phases_.insert(phases_.end(), other.phases_.begin(), other.phases_.end());
  phase_types_.insert(phase_types_.end(), other.phase_types_.begin(),
                      other.phase_types_.end());
  for (int i = offset; i < phases_.size(); ++i) {
    predecessor = i > 0 ? phases_[i - 1].back() : start_state_;
    auto dist_offset = predecessor.dist_ - phases_[i][0].dist_;
    for (int j = 0; j < phases_[i].size(); ++j) {
      phases_[i][j].dist_ = dist_offset + phases_[i][j].dist_;
    }
  }
  fix_drive(offset > 0 ? offset - 1 : 0);
  return *this;
}
void train_drive::erase_elements(int const& offset, const int& to_delete) {
  auto first_phase = phases_.begin() + offset;
  auto first_type = phase_types_.begin() + offset;
  phases_.erase(first_phase, first_phase + to_delete);
  phase_types_.erase(first_type, first_type + to_delete);
}

void train_drive::insert(const int& offset, const phases& new_phases,
                         vector<phase_type> const& types) {
  phases_.insert(phases_.begin() + offset, new_phases.begin(),
                 new_phases.end());
  phase_types_.insert(phase_types_.begin() + offset, types.begin(),
                      types.end());
}

void train_drive::print() {
  std::cout << "Train Drive Print" << std::endl;
  utls::for_each(phase_types_, [](phase_type const& type) {
    std::cout << type << std::endl;
  });
  std::cout << std::endl;
  for (auto const& phase : phases_) {
    utls::for_each(phase, [](train_state const& state) {
      std::cout << state.time_ << ' ' << state.dist_ << ' ' << state.speed_
                << std::endl;
    });
  }
}
// gets the interval the distance is in
// theoretically, this information could be saved in the drive itself
interval get_interval(si::length distance) {
  auto it =
      utls::find_if(intr_points, [distance](interval_point const& int_point) {
        return int_point.distance_ >= distance;
      });
  utls::sassert(it != intr_points.end(), "Distance of {} isnt in the intervals",
                distance);
  utls::sassert(
      it + 1 != intr_points.end() || it->distance_ > distance,
      "Distance is at last interval point, no interval starts with it");
  return it->distance_ == distance ? interval{&*it, &*(it + 1)}
                                   : interval{&*(it - 1), &*it};
}

void increase_time(train_drive& drive,
                   train_path_envelope::tpe_point const& point,
                   rs::train_physics const& tp,
                   vector<interval_point> const& interval_points,
                   std::function<int(train_drive const&)> const& get_offset) {
  utls::sassert(!drive.phases_.empty(), "increase time got empty drive");
  utls::sassert(drive.phases_.back().back().time_ < point.e_time_,
                "increase time not needed");
  set_pt(point);
  set_intervals(interval_points);
  while (drive.phases_.size() >= 3) {
    auto offset = get_offset(drive);
    auto t1 = drive.phase_types_[offset];
    auto t2 = drive.phase_types_[offset + 1];
    auto t3 = offset + 2 < drive.phase_types_.size()
                  ? drive.phase_types_[offset + 2]
                  : invalid;
    if (AHD_checker(t1, t2, t3)) {
      if (check_AHD(drive, offset)) return;
      continue;
    }
    if (AHA_checker(t1, t2, t3)) {
      if (check_AHA(drive, tp, offset)) return;
      continue;
    }
    if (HDH_checker(t1, t2, t3)) {
      if (check_HDH(drive, tp, offset)) return;
      continue;
    }
    if (DHA_checker(t1, t2, t3)&&check_DHA(drive, tp, offset)) return;
  }
  auto HA_checker = [](phase_type t1, phase_type t2) {
    return t1 == cruising && t2 == acceleration;
  };
  auto DA_checker = [](phase_type t1, phase_type t2) {
    return t1 == braking && t2 == acceleration;
  };
  auto DH_checker = [](phase_type t1, phase_type t2) {
    return t1 == braking && t2 == cruising;
  };
  while (drive.phases_.size() == 2) {
    auto t1 = drive.phase_types_.front();
    auto t2 = drive.phase_types_.back();
    if (AHD_checker(t1, t2, invalid)) {
      if (check_AHD(drive, 0)) return;
      continue;
    }
    if (AHA_checker(t1, t2, invalid)) {
      if (check_AHA(drive, tp, 0)) return;
      continue;
    }
    if (HDH_checker(t1, t2, invalid)) {
      if (check_HDH(drive, tp, 0)) return;
      continue;
    }
    if (HA_checker(t1, t2)) {
      if (check_HA(drive, tp)) return;
      continue;
    }
    if (DA_checker(t1, t2)) {
      if (check_DA(drive, tp)) return;
      continue;
    }
    if (DH_checker(t1, t2)) {
      check_DH(drive, tp);
      return;
    }
  }
  if(drive.phase_types_.front()==acceleration&&check_A(drive,tp)) return;
  if(drive.phase_types_.front()==cruising) {
    check_H(drive,tp);
    return;
  }
  if(drive.phase_types_.front()==braking) check_D(drive);
}

train_state find_state_with_speed(si::speed const& speed,
                                  vector<train_state> const& states,
                                  bool const& accel) {
  auto it = utls::find_if(states, [speed, accel](train_state const& state) {
    return accel ? state.speed_ >= speed : state.speed_ <= speed;
  });
  utls::sassert(it != states.end(), "no state with speed {} found", speed);
  return it->speed_ == speed
             ? *it
             : rk4::detail::get_intersection_at_speed(speed, *(it - 1), *it);
}
std::tuple<phases, types> make_new_phases(phases const& inserted_phases,
                                          types const& inserted_types) {
  phases result_phases;
  types result_types;
  for (auto i = 0; i < inserted_phases.size(); ++i) {
    auto& phase = inserted_phases[i];
    if (phase.size() < 2) continue;
    result_phases.push_back(phase);
    result_types.push_back(inserted_types[i]);
  }
  return {result_phases, result_types};
}
void make_result_AHD(train_drive& drive, int const& offset,
                     vector<train_state>& accel_phase,
                     vector<train_state>& brake_phase,
                     si::speed const& cruise_v, train_state const& end_cruise,
                     train_state const& end_accel) {
  int to_delete = drive.phase_types_[offset + 1] == braking ? 2 : 3;
  drive.erase_elements(offset, to_delete);
  std::erase_if(brake_phase, [cruise_v](train_state const& state) {
    return state.speed_ >= cruise_v;
  });
  std::erase_if(accel_phase, [cruise_v](train_state const& state) {
    return state.speed_ >= cruise_v;
  });
  vector<train_state> new_cruise{end_accel, end_cruise};
  accel_phase.push_back(end_accel);
  brake_phase.insert(brake_phase.begin(), end_cruise);
  auto [phases, types] = make_new_phases({accel_phase, new_cruise, brake_phase},
                                         {acceleration, cruising, braking});
  drive.insert(offset, phases, types);
  drive.fix_drive(offset==0?offset:offset-1);
}

bool check_cruise_speed_sufficient(train_state start_state,train_state end_state,si::time planned_dif) {
  auto t_old = end_state.time_-start_state.time_;
  return get_cruise_time(start_state,end_state)-t_old>=planned_dif;
}

vector<train_state> search_for_cruise_start(vector<train_state> accel_phase,si::speed speed,vector<train_state> brake_phase,si::time planned_dif) {
  for(int i=accel_phase.size()-2; i>=0&&accel_phase[i].speed_>=speed; --i) {
    auto end_cruise = find_state_with_speed(accel_phase[i].speed_,brake_phase,false);
    if(check_cruise_speed_sufficient(accel_phase[i],end_cruise,planned_dif)) {
      return {accel_phase[i],end_cruise};
    }
  }
  auto start_cruise = find_state_with_speed(speed,accel_phase,true);
  auto end_cruise = find_state_with_speed(speed,brake_phase,false);
  return {start_cruise,end_cruise};
}

// das hier sollte klappen
bool check_AHD(train_drive& drive, int const& offset) {
  bool first_is_accel = drive.phase_types_[offset] ==acceleration;
  bool second_is_braking = drive.phase_types_[offset+1] == braking;
  auto third_type = offset+2<drive.phase_types_.size()?drive.phase_types_[offset+2]:invalid;
  utls::sassert(first_is_accel &&
                  (second_is_braking || third_type == braking),
              "AHD got wrong types");
  // several values needed for the calculations
  auto brake_phase = second_is_braking ? drive.phases_[offset+1] : drive.phases_[offset+2];
  auto accel_phase = drive.phases_[offset];
  auto min_cruise_v =
      std::max(accel_phase.front().speed_, brake_phase.back().speed_);
  auto end_accel = accel_phase.front().speed_ >= brake_phase.back().speed_
                       ? accel_phase.front()
                       : find_state_with_speed(min_cruise_v, accel_phase, true);
  auto end_new_cruise =
      brake_phase.back().speed_ >= accel_phase.front().speed_
          ? brake_phase.back()
          : find_state_with_speed(min_cruise_v, brake_phase, false);
  auto distance = end_new_cruise.dist_ - end_accel.dist_;
  auto t_old = end_new_cruise.time_ - end_accel.time_;
  auto time_dif = distance / min_cruise_v - t_old;
  auto t_real = drive.phases_.back().back().time_;
  auto planned_dif = pt.e_time_ - t_real;
  if (time_dif <= planned_dif) {
    make_result_AHD(drive, offset, accel_phase, brake_phase, min_cruise_v,
                    end_new_cruise, end_accel);
    return distance / min_cruise_v - t_old == pt.e_time_ - t_real;
  }
  auto cruise_phase = search_for_cruise_start(accel_phase,min_cruise_v,brake_phase,planned_dif);
  make_result_AHD(drive,offset,accel_phase,brake_phase,cruise_phase.front().speed_,cruise_phase.back(),cruise_phase.front());
  return true;
}
// sollte klappen
bool check_AHA(train_drive& drive, rs::train_physics const& tp,
               int const& offset) {
  utls::sassert(drive.phase_types_[offset] == acceleration &&
                    drive.phase_types_[offset + 1] == cruising,
                "Wrong types for AHA");
  auto planned_dif = pt.e_time_-drive.phases_.back().back().time_;
  train_state state = drive.phases_[offset + 1].back();
  auto accel_phase = drive.phases_[offset];
  bool result_found = false;
  train_state accel_state;
  vector<train_state> backwards_accel_states{state};
  auto lowest_speed = accel_phase.front().speed_;
  while (state.speed_ != lowest_speed) {
    auto delta = rk4::rk4_step(state.speed_, rk4::delta_t,
                               get_interval(state.dist_).slope(), tp);
    utls::sassert(delta.speed_.is_positive(),"No positive delta speed");
    if (state.speed_ - delta.speed_ < lowest_speed) {
      auto speed_dif = state.speed_ - lowest_speed;
      auto factor = speed_dif / delta.speed_;
      delta.speed_ = speed_dif;
      delta.dist_ = delta.dist_ * factor;
      delta.time_ = delta.time_ * factor;
    }
    state -= delta;
    backwards_accel_states.push_back(state);
    accel_state =
        find_state_with_speed(state.speed_, accel_phase, true);
    if (check_cruise_speed_sufficient(accel_state,state,planned_dif)) {
      result_found = true;
      break;
    }
  }
  std::erase_if(accel_phase, [accel_state](train_state const& phase_state) {
    return phase_state.speed_ >= accel_state.speed_;
  });
  drive.erase_elements(offset, 2);
  std::ranges::reverse(backwards_accel_states);
  vector<train_state> new_cruise{accel_state, state};
  accel_phase.push_back(accel_state);
  auto [phases, types] =
      make_new_phases({accel_phase, new_cruise, backwards_accel_states},
                      {acceleration, cruising, acceleration});
  drive.phases_.insert(drive.phases_.begin() + offset, phases.begin(),
                       phases.end());
  drive.phase_types_.insert(drive.phase_types_.begin() + offset, types.begin(),
                            types.end());
  drive.fix_drive(offset==0?0:offset-1);
  return result_found;
}

si::speed find_optimal_speed(si::speed start_speed,si::accel accel,si::length start_dist,si::length end_dist,si::time time) {
  utls::sassert(!accel.is_zero(),"Acceleration is zero");
  // p and q from the p q formula
  auto p = start_speed+accel*time;
  auto square_root_val = (p.pow<2>()-start_speed.pow<2>()-2*accel*(end_dist-start_dist));
  utls::sassert(!square_root_val.is_negative(),"Value in square root is negative");
  return accel.is_negative()?p+square_root_val.sqrt():p-square_root_val.sqrt();
}

vector<train_state> continue_brake(train_state& end_brake,rs::train_physics const& tp,
  si::speed const& min_speed,si::time const& target_time,si::length const& max_dist) {
  utls::sassert(end_brake.speed_>=min_speed,"Attempt to brake to higher speed");
  auto interval = get_interval(end_brake.dist_);
  vector<train_state> result{end_brake};
  while(end_brake.speed_>min_speed&&end_brake.dist_<max_dist) {
    if(interval.length().is_zero()) {
      ++interval;
      continue;
    }
    auto deaccel = tp.braking_deaccel(
      interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
    //The first state may not be on the interval border
    auto length = interval.length()-(end_brake.dist_-interval.start_distance());
    end_brake = rk4::brake_over_distance_with_target(end_brake,deaccel,length,min_speed);
    result.push_back(end_brake);
    auto end_cruise_time = end_brake.time_+get_cruise_time(end_brake.speed_,end_brake.dist_,max_dist);
    if(end_cruise_time==target_time) {
      return result;
    }
    if(end_cruise_time>target_time) {
      result.pop_back();
      auto state = result.back();
      auto cruise_speed = find_optimal_speed(state.speed_,deaccel,state.dist_,max_dist,target_time-state.time_);
      auto brake_delta = rk4::brake(state.speed_,cruise_speed,deaccel);
      end_brake = state+brake_delta;
      end_brake.speed_ = brake_delta.speed_;
      result.push_back(end_brake);
      utls::sassert(end_brake.time_+get_cruise_time(cruise_speed,end_brake.dist_,max_dist)==target_time,"find optimal speed returned wrong value");
      return result;
    }
    ++interval;
  }
  return result;
}

bool check_HDH(train_drive& drive, rs::train_physics const& tp,
               int const& offset) {
  utls::sassert(drive.phase_types_[offset] == cruising &&
                    drive.phase_types_[offset + 1] == braking,
                "Wrong types for HDH");
  auto cruise_phase = drive.phases_[offset];
  auto brake_phase = drive.phases_[offset+1];
  auto planned_dif = pt.e_time_-drive.phases_.back().back().time_;
  auto total_time = brake_phase.back().time_+planned_dif-cruise_phase.front().time_;
  auto brake_time = brake_phase.back().time_-brake_phase.front().time_;
  auto cruise_length = cruise_phase.back().dist_-cruise_phase.front().dist_;
  auto v_opt = cruise_length/(total_time-brake_time);
  v_opt = std::max(v_opt,brake_phase.back().speed_);
  drive.erase_elements(offset, 2);
  auto new_brake = continue_brake(cruise_phase.front(),tp,v_opt,si::time::infinity(),si::length::infinity());
  auto end_cruise = find_state_with_speed(v_opt,brake_phase,false);
  std::erase_if(brake_phase, [v_opt](train_state const& state) {return state.speed_>=v_opt;});
  brake_phase.insert(brake_phase.begin(),end_cruise);
  end_cruise.time_ = new_brake.back().time_+get_cruise_time(new_brake.back(),end_cruise);
  vector<train_state> new_cruise{new_brake.back(),end_cruise};
  auto [new_phases,new_types] =
    make_new_phases({new_brake,new_cruise,brake_phase},{braking,cruising,braking});
  drive.insert(offset,new_phases,new_types);
  drive.fix_drive(offset==0?0:offset-1);
  return drive.phases_.back().back().time_>=pt.e_time_;
}


bool check_DHA(train_drive& drive, rs::train_physics const& tp,
               int const& offset) {
  throw utl::fail("not implemented DHA");
}

train_state find_end_of_new_acceleration(
    vector<train_state> const& accel_phase) {
  int i;
  for (i = accel_phase.size() - 2; i >= 0 && accel_phase[i].speed_ >= pt.v_min_;
       --i) {
    auto cruise_time = get_cruise_time(accel_phase[i].speed_,
                                       accel_phase[i].dist_, pt.distance_);
    if (accel_phase[i].time_ + cruise_time >= pt.e_time_) return accel_phase[i];
  }
  return i == -1 ? accel_phase.front()
                 : find_state_with_speed(pt.v_min_, accel_phase, true);
}
void make_result_HA_DA(train_state const& end_accel, train_drive& drive) {
  std::erase_if(drive.phases_.back(), [end_accel](train_state const& state) {
    return state.speed_ >= end_accel.speed_;
  });
  drive.phases_.back().push_back(end_accel);
  auto end_time =
      end_accel.time_ +
      get_cruise_time(end_accel.speed_, end_accel.dist_, pt.distance_);
  train_state end_new_cruise(end_time, pt.distance_, end_accel.speed_);
  vector<train_state> new_cruise{end_accel, end_new_cruise};
  auto [new_phases, new_types] = make_new_phases(
      {drive.phases_.back(), new_cruise}, {acceleration, cruising});
  drive.phases_.pop_back();
  drive.phase_types_.pop_back();
  drive.phases_.insert(drive.phases_.end(), new_phases.begin(),
                       new_phases.end());
  drive.phase_types_.insert(drive.phase_types_.end(), new_types.begin(),
                            new_types.end());
  drive.fix_phases(0);
}

bool check_HA(train_drive& drive, rs::train_physics const& tp) {
  utls::sassert(drive.phases_.size() == 2,
                "Method for 2 types called while there were more types");
  utls::sassert(drive.phase_types_.front() == cruising &&
                    drive.phase_types_.back() == acceleration,
                "Wrong types");
  auto const& accel_phase = drive.phases_.back();
  int i;
  for (i = accel_phase.size() - 2; i >= 0 && accel_phase[i].speed_ >= pt.v_min_;
       --i) {
    if (accel_phase[i].time_ + get_cruise_time(accel_phase[i].speed_,
                                               accel_phase[i].dist_,
                                               pt.distance_) <
        pt.e_time_)
      continue;
    make_result_HA_DA(accel_phase[i], drive);
    return true;
  }
  train_state end_accel =
      i == -1 ? accel_phase.front()
              : find_state_with_speed(pt.v_min_, accel_phase, true);
  make_result_HA_DA(end_accel, drive);
  if (drive.phases_.back().back().time_ >= pt.e_time_) return true;
  if (drive.phase_types_.size() == 1) return false;
  if (drive.phases_.size() == 3 && check_AHA(drive, tp, 1)) return true;
  return slowest_drive(drive, 0, tp);
}


bool check_DH(train_drive& drive,rs::train_physics const& tp){
  auto brake_end = drive.phases_.front().back();
  auto continued_brake = continue_brake(brake_end,tp,pt.v_min_,pt.e_time_,pt.distance_);
  auto end_time = brake_end.time_+get_cruise_time(brake_end.speed_,brake_end.dist_,pt.distance_);
  if(end_time<pt.e_time_&&brake_end.dist_==pt.distance_) throw std::logic_error("even slowest drive is too fast");
  auto& original_brake = drive.phases_.front();
  original_brake.insert(original_brake.end(),continued_brake.begin()+1,continued_brake.end());
  drive.phases_.pop_back();
  drive.phase_types_.pop_back();
  if(brake_end.dist_<pt.distance_) {
    train_state end_cruise(end_time,pt.distance_,brake_end.speed_);
    drive.push_back({brake_end,end_cruise},cruising);
  }
  auto float_precision = si::time(FP_PRECISION<si::time::precision>);
  if(end_time<pt.e_time_&&(pt.e_time_-end_time)>=float_precision) slowest_drive(drive,1,tp);
  return true;
}

bool check_DA(train_drive& drive, rs::train_physics const& tp) {
  utls::sassert(drive.phases_.size() == 2,
                "Method for 2 types called while there were more types");
  utls::sassert(drive.phase_types_.front() == braking &&
                    drive.phase_types_.back() == acceleration,
                "Wrong types");
  utls::sassert(drive.phases_.back().back().speed_ > pt.v_min_,
                "Even slowest drive is too fast");
  auto end_accel = find_end_of_new_acceleration(drive.phases_.back());
  make_result_HA_DA(end_accel, drive);
  auto& end_brake = drive.phases_.front().back();
  if (end_brake.speed_ >= pt.v_min_)
    return drive.phases_.back().back().time_ >= pt.e_time_;
  if (check_AHA(drive, tp, 1)) return true;
  return slowest_drive(drive, 1, tp);
}

bool check_A(train_drive& drive,rs::train_physics const& tp) {
  utls::sassert(drive.phase_types_ == vector<phase_type>{acceleration},"Wrong types in check_A");
  auto& phase = drive.phases_.front();
  train_state end_accel;
  bool state_changed = false;
  int i;
  for (i = phase.size() - 2; i >= 0; --i) {
    end_accel = phase[i];
    if (end_accel.speed_ < pt.v_min_) {
      end_accel = find_state_with_speed(pt.v_min_, drive.phases_.front(), true);
      state_changed = true;
    }
    if (i == 0 || end_accel.speed_ == pt.v_min_||end_accel.time_ + get_cruise_time(end_accel.speed_,end_accel.dist_,pt.distance_) >=
                      pt.e_time_) {
      break;
    }
  }
  if(end_accel.speed_.is_zero()) {
    auto copy_pt = pt;
    throw std::logic_error("check_a created cruise of 0 speed");
  }
  drive.phases_.front().erase(drive.phases_.front().begin() + i + 1,
                                  drive.phases_.front().end());
  if(state_changed) drive.phases_.front().push_back(end_accel);
  train_state end_state(
      end_accel.time_ + get_cruise_time(end_accel.speed_,end_accel.dist_,pt.distance_),
      pt.distance_, end_accel.speed_);
  auto cruise_phase = {end_accel, end_state};
  if (drive.phases_.front().size() == 1) {
    drive.phases_.pop_back();
    drive.phase_types_.pop_back();
  }
  drive.phases_.emplace_back(cruise_phase);
  drive.phase_types_.push_back(cruising);
  auto finished = end_state.time_>=pt.e_time_;
  if(finished||drive.phases_.size()==1) return finished;
  if(check_AHA(drive,tp,0)) return true;
  return check_HA(drive,tp);
}

bool check_H(train_drive& drive,rs::train_physics const& tp) {
  auto brake_state = drive.phases_.front().front();
  auto new_brake = continue_brake(brake_state,tp,pt.v_min_,pt.e_time_,pt.distance_);
  auto end_time = brake_state.time_+get_cruise_time(brake_state.speed_,brake_state.dist_,pt.distance_);
  if(end_time<pt.e_time_&&brake_state.dist_==pt.distance_) {
    throw std::logic_error("tpe not drivable");
  }
  drive.erase_elements(0,1);
  drive.push_back(new_brake,braking);
  if(brake_state.dist_<pt.distance_) {
    train_state end_cruise(end_time,pt.distance_,brake_state.speed_);
    drive.push_back({brake_state,end_cruise},cruising);
  }
  auto float_precision = si::time(FP_PRECISION<si::time::precision>);
  if(end_time<pt.e_time_&&(pt.e_time_-end_time)>=float_precision) slowest_drive(drive,1,tp);
  return true;
}
bool check_D(train_drive& drive) {
  utls::sassert(drive.phases_.size()==1&&drive.phase_types_.front()==braking,
    "Wrong method");
  if (drive.phases_.back().back().time_ >= pt.e_time_)
    throw std::logic_error(
        "Train is already slow enough, no use in calling this");
  if(drive.phases_.back().back().speed_.is_zero()) {
    drive.phases_.back().back().time_ = pt.e_time_;
    return true;
  }
  throw std::logic_error(
      "Train takes too short even with constant braking. This should never "
      "happen.");
}

bool slowest_drive(train_drive& drive, int const& cruise_index,
                   rs::train_physics const& tp) {
  auto float_precision = si::time(FP_PRECISION<si::time::precision>);
  if(drive.phases_.back().back().time_ >= pt.e_time_||(pt.e_time_-drive.phases_.back().back().time_)<float_precision) {
    return true;
  }
  throw utl::fail("slowest_drive not implemented yet");
}

}  // namespace increase_time