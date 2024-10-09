#include "soro/runtime/common/increase_time.h"
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/physics/rk4/detail/rk4_step.h"
#include "soro/runtime/physics/rk4/brake.h"
#include "soro/rolling_stock/train_physics.h"
#include "soro/runtime/physics/rk4/detail/delta_t.h"
#include "soro/runtime/physics/rk4/detail/get_intersection.h"
#include "soro/runtime/common/interval.h"
#include "soro/runtime/common/phase_checkers.h"
#pragma once
namespace increase_time {
using namespace soro;
using namespace soro::runtime;
train_path_envelope::tpe_point pt;
vector<interval_point> intr_points;


void set_pt(train_path_envelope::tpe_point const& point){
  pt.distance_ = point.distance_;
  pt.e_time_ = point.e_time_;
  pt.l_time_ = point.l_time_;
  pt.v_min_ = point.v_min_;
  pt.v_max_ = point.v_max_;
}

void set_intervals(vector<interval_point> const& interval_points){
  intr_points.erase(intr_points.begin(),intr_points.end());
  intr_points.insert(intr_points.begin(),interval_points.begin(),interval_points.end());
}

si::time get_cruise_time(si::speed const& speed, si::length const& start,
                         si::length const& stop) {
  return (stop - start) / speed;
}
si::time get_cruise_time(train_state const& start, train_state const&  end) {
  utls::sassert(start.speed_ == end.speed_,
              "Cruise time demanded for non cruise");
  return get_cruise_time(start.speed_, start.dist_, end.dist_);
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
    auto predecessor = i>0?phases_[i-1].back():start_state_;
    switch (phase_types_[i]) {
      case cruising:
        fix_times_cruise(phases_[i], predecessor);
        break;
      case braking:
      case acceleration:
        fix_times_non_cruise(phases_[i], predecessor);
        break;
      default: throw std::logic_error("Invalid phase type detected");
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
  fix_phases(offset);
  fix_times(offset);
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

void train_drive::push_back(vector<runtime::train_state>const& phase, phase_type const& type) {
  phases_.push_back(phase);
  phase_types_.push_back(type);
}

train_drive train_drive::operator+=(const train_drive& other) {
  if(other.phases_.empty()) return *this;
  auto offset = phases_.size();
  train_state predecessor;
  phases_.insert(phases_.end(),other.phases_.begin(),other.phases_.end());
  phase_types_.insert(phase_types_.end(),other.phase_types_.begin(),other.phase_types_.end());
  for(int i=offset;i<phases_.size();++i){
    predecessor = i>0?phases_[i-1].back():start_state_;
    auto dist_offset = predecessor.dist_-phases_[i][0].dist_;
    for(int j=0;j<phases_[i].size();++j) {
      phases_[i][j].dist_=dist_offset+phases_[i][j].dist_;
    }
  }
  fix_drive(offset>0?offset-1:0);
  return *this;
}
void train_drive::erase_elements(int const& offset, const int& to_delete) {
  auto first_phase = phases_.begin()+offset;
  auto first_type = phase_types_.begin()+offset;
  phases_.erase(first_phase, first_phase + to_delete);
  phase_types_.erase(first_type, first_type + to_delete);
}

void train_drive::insert(const int& offset, const phases& new_phases, vector<phase_type> const& types) {
  phases_.insert(phases_.begin()+offset,new_phases.begin(),new_phases.end());
  phase_types_.insert(phase_types_.begin()+offset,types.begin(),types.end());
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
/**
 * main method to increase the time of a ride by changing the speed profile
 * @param drive the ride
 */
void increase_time(train_drive& drive,
                   train_path_envelope::tpe_point const& point,
                   soro::rs::train_physics const& tp,
                   vector<interval_point> const& interval_points,
                   std::function<int(train_drive const&)> const& get_offset) {
  set_pt(point);
  set_intervals(interval_points);
  while (drive.phases_.size() >= 3) {
    // while(offset<drive.phase_types_.size()-1){
    auto offset = get_offset(drive);
    auto t1 = drive.phase_types_[offset];
    auto t2 = drive.phase_types_[offset+1];
    auto t3 = offset+2<drive.phase_types_.size()?drive.phase_types_[offset+2]:invalid;
    if (AHD_checker(t1, t2, t3)) {
      if (check_AHD(drive,offset)) return;
      continue;
    }
    if (AHA_checker(t1, t2, t3)) {
      if (check_AHA(drive,tp,offset)) return;
      continue;
    }
    if (HDH_checker(t1, t2, t3)) {
      if (check_HDH(drive, tp,offset)) return;
      continue;
    }
    if (DHA_checker(t1, t2, t3)) {
      if (check_DHA(drive, tp,offset)) return;
      continue;
    }
    //}
  }
  throw std::logic_error("two phase methods not yet implemented");
  /*auto HA_checker = [](phase_type t1,phase_type t2){return
  t1==cruising&&t2==acceleration;}; auto DA_checker = [](phase_type
  t1,phase_type t2){return t1==braking&&t2==acceleration;}; auto DH_checker =
  [](phase_type t1,phase_type t2){return t1==braking&&t2==cruising;};
  while(drive.phases_.size()==2){
    auto t1 = drive.phase_types_.front();
    auto t2 = drive.phase_types_.back();
    if(AHD_checker(t1,t2,invalid)){
     if(check_AHD(drive))return;
     continue;
    }
    if(AHA_checker(t1,t2,invalid)){
      if(check_AHA(drive))return;
      continue;
    }
    if(HDH_checker(t1,t2,invalid)){
      if(check_HDH(drive))return;
      continue;
    }
    if(HA_checker(t1,t2)){
      if(check_HA(drive)) return;
      continue;
    }
    if(DA_checker(t1,t2)){
      if(check_DA(drive))return;
      continue;
    }
    if(DH_checker(t1,t2)){
      if(check_DH(drive)) return;
      continue;
    }
  }
  if(drive.phase_types_.front()==acceleration&&check_A(drive))return;
  if(drive.phase_types_.front()==cruising&&check_H(drive))return;
  if(drive.phase_types_.front()==braking) check_D(drive);*/
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
void make_result_AHD(train_drive& drive,int const& offset,vector<train_state>& accel_phase,vector<train_state>& brake_phase,si::speed const& cruise_v,train_state const& end_cruise,train_state const& end_accel){
  int to_delete = drive.phase_types_[offset+1]==braking?2:3;
  drive.erase_elements(offset,to_delete);
  std::erase_if(brake_phase,[cruise_v](train_state const& state){return state.speed_>=cruise_v;});
  std::erase_if(accel_phase,[cruise_v](train_state const& state){return state.speed_>=cruise_v;});
  vector<train_state> new_cruise{end_accel,end_cruise};
  accel_phase.push_back(end_accel);
  brake_phase.insert(brake_phase.begin(),end_cruise);
  auto [phases,types] = make_new_phases({accel_phase,new_cruise,brake_phase},{acceleration,cruising,braking});
  drive.insert(offset,phases,types);
  drive.fix_drive(offset);
}
// das hier sollte klappen
bool check_AHD(train_drive& drive, int const& offset) {
  auto phase_it = drive.phases_.begin() + offset;
  auto type_it = drive.phase_types_.begin() + offset;
  bool second_is_braking = *(type_it + 1) == braking;
  utl::verify(*type_it==acceleration&&(second_is_braking||*(type_it+2)==braking),"AHD got wrong types");
  // several values needed for the calculations
  auto t_real = drive.phases_.back().back().time_;
  auto brake_phase = second_is_braking ? *(phase_it + 1) : *(phase_it + 2);
  auto accel_phase = *phase_it;
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
  if (distance/min_cruise_v-t_old<=pt.e_time_-t_real) {
    make_result_AHD(drive,offset,accel_phase,brake_phase,min_cruise_v,end_new_cruise,end_accel);
    return distance/min_cruise_v-t_old==pt.e_time_-t_real;
  }
  // hier könnte es passieren, dass der letzte angeschaute zustand in acceleration auch zu schnell ist dann würde ne exception geworfen werden. Hoffe nicht.
  for (int i = accel_phase.size() - 2;
       i >= 0 && accel_phase[i].speed_ >= min_cruise_v; --i) {
    end_new_cruise =
        find_state_with_speed(accel_phase[i].speed_, brake_phase, false);
    distance = end_new_cruise.dist_ - accel_phase[i].dist_;
    if (distance / accel_phase[i].speed_ + accel_phase[i].time_ -
            end_new_cruise.time_ >=
        pt.e_time_ - t_real) {
      make_result_AHD(drive,offset,accel_phase,brake_phase,accel_phase[i].speed_,end_new_cruise,accel_phase[i]);
      return true;
    }
  }
  throw std::logic_error("check_AHD should not get here");
}
// sollte klappen
bool check_AHA(train_drive& drive, rs::train_physics const& tp,
               int const& offset) {
  utls::sassert(drive.phase_types_[offset] == acceleration &&
                    drive.phase_types_[offset + 1] == cruising,
                "Wrong types for AHA");
  auto t_real = drive.phases_.back().back().time_;
  train_state state = drive.phases_[offset+1].back();
  bool result_found = false;
  train_state accel_state;
  vector<train_state> backwards_accel_states{state};
  auto lowest_speed = drive.phases_[offset].front().speed_;
  while (state.speed_ != lowest_speed) {
    auto delta = rk4::rk4_step(state.speed_, rk4::delta_t,
                               get_interval(state.dist_).slope(), tp);
    if (state.speed_ - delta.speed_ < lowest_speed) {
      auto speed_dif = state.speed_ - lowest_speed;
      auto factor = speed_dif / delta.speed_;
      delta.speed_ = speed_dif;
      delta.dist_ = delta.dist_ * factor;
      delta.time_ = delta.time_ * factor;
    }
    state -= delta;
    backwards_accel_states.push_back(state);
    accel_state = find_state_with_speed(state.speed_, drive.phases_[offset], true);
    if ((state.dist_ - accel_state.dist_) / state.speed_ - state.time_ +
            accel_state.time_ >=
        pt.e_time_ - t_real) {
      result_found = true;
      break;
    }
  }
  auto accel_phase = drive.phases_[offset];
  std::erase_if(accel_phase, [accel_state](train_state const& state) {
    return state.speed_ >= accel_state.speed_;
  });
  int to_delete = 2;
  drive.erase_elements(offset,to_delete);
  std::reverse(backwards_accel_states.begin(), backwards_accel_states.end());
  vector<train_state> new_cruise{accel_state, state};
  accel_phase.push_back(accel_state);
  auto [phases,types] = make_new_phases({accel_phase,new_cruise,backwards_accel_states},{acceleration,cruising,acceleration});
  drive.phases_.insert(drive.phases_.begin()+offset,phases.begin(),phases.end());
  drive.phase_types_.insert(drive.phase_types_.begin()+offset,types.begin(),types.end());
  /*if (accel_phase.empty()) {
    drive.phases_.insert(drive.phases_.begin() + offset, new_cruise);
    drive.phases_.insert(drive.phases_.begin() + offset,
                         backwards_accel_states);
    drive.phase_types_.insert(drive.phase_types_.begin() + offset, cruising);
    drive.phase_types_.insert(drive.phase_types_.begin() + offset,
                              acceleration);
  } else {
    accel_phase.push_back(accel_state);
    drive.phases_.insert(drive.phases_.begin() + offset,
                         {accel_phase, new_cruise, backwards_accel_states});
    drive.phase_types_.insert(drive.phase_types_.begin() + offset,
                              {acceleration, cruising, acceleration});
  }*/
  drive.fix_drive(offset);
  return result_found;
}
/**
 * tries to change part of a ride to make the ride take long enough.
 * It only inspects a part of the ride that has a H-D-H Phase-chain
 * @param drive the ride
 * @return true iff the methode finds an alteration that increases the time enough
 */
// sollte klappen
bool check_HDH(train_drive& drive, rs::train_physics const& tp,
               int const& offset) {
  utls::sassert(drive.phase_types_[offset] == cruising &&
                    drive.phase_types_[offset + 1] == braking,
                "Wrong types for HDH");
  auto phase_it = drive.phases_.begin() + offset;
  auto types_it = drive.phase_types_.begin() + offset;
  auto initial = phase_it->front();
  auto interval = get_interval(initial.dist_);
  auto brake_end_speed = (phase_it + 1)->back().speed_;
  vector<train_state> brake = {initial};
  auto t_real = drive.phases_.back().back().time_;
  bool result_found = false;
  train_state end_of_new_cruise;
  while (initial.speed_ != brake_end_speed) {
    if(interval.length().is_zero()){
      ++interval;
      continue;
    }
    auto deaccel =
        tp.braking_deaccel(interval.infra_limit(), interval.bwp_limit(),
                           interval.brake_path_length());
    utls::sassert(deaccel.is_negative(), "deaccel isnt negative");
    initial = rk4::brake_over_distance_with_target(
        initial, deaccel, interval.length(), brake_end_speed);
    ++interval;
    brake.push_back(initial);
    if (initial.speed_ == brake_end_speed)
      end_of_new_cruise =
          phase_it + 2 >= drive.phases_.end() || *(types_it + 2) != cruising
              ? (phase_it + 1)->back()
              : (phase_it + 2)->back();
    else
      end_of_new_cruise =
          find_state_with_speed(initial.speed_, *(phase_it + 1), false);
    /*if(end_of_new_cruise.dist_==initial.dist_){
      for(auto const& phase: drive.phases_){
        utls::for_each(phase,[](train_state const& state){std::cout<<state.dist_<<" "<<state.speed_<<std::endl;});
        std::cout<<std::endl;
      }
      utls::for_each(drive.phase_types_,[](phase_type const& type){std::cout<<type<<std::endl;});
      std::cout<<std::endl;
      utls::for_each(intr_points,[](interval_point const& point){std::cout<<point.distance_<<std::endl;});
      std::cout<<std::endl;
      std::cout<<pt.distance_;
      throw std::logic_error("new cruise ends where it begins");
    }*/
    if(initial.time_+ get_cruise_time(initial,end_of_new_cruise)-end_of_new_cruise.time_>=pt.e_time_ - t_real) {
      result_found = true;
      break;
    }
  }
  auto brake_phase = *(phase_it + 1);
  std::erase_if(brake_phase, [initial](train_state const& state) {
    return state.speed_ >= initial.speed_;
  });
  brake_phase.insert(brake_phase.begin(),end_of_new_cruise);
  drive.erase_elements(offset,2);
  vector<train_state> new_cruise({initial, end_of_new_cruise});
  auto new_phases = vector<vector<train_state>>{brake, new_cruise, brake_phase};
  auto new_types = vector<phase_type>{braking,cruising,braking};
  std::tie(new_phases,new_types) = make_new_phases(new_phases,new_types);
  drive.phases_.insert(drive.phases_.begin() + offset, new_phases.begin(),
                       new_phases.end());
  drive.phase_types_.insert(drive.phase_types_.begin() + offset,
                            new_types.begin(), new_types.end());
  drive.fix_drive(offset);
  return result_found;
}
/**
 *
 * @param drive
 * @return
 */
bool check_DHA(train_drive& drive, rs::train_physics const& tp,
               int const& offset) {
  throw std::logic_error("not implemented DHA");
  /*auto brake_phase = *phase_it;
  auto accel_phase = *(phase_it+2);
  auto brake_state = brake_phase.back();
  auto accel_state = accel_phase.front();
  return true;*/
}
/**
 *
 * @param drive
 * @return
 */
// noch nicht richtig
/*bool check_HA(train_drive& drive){
  train_drive new_drive;
  new_drive.phases_ = {drive.phases_.back()};
  new_drive.phase_types_ = {acceleration};
  return check_A(new_drive);
}*/
/**
 *
 * @param drive
 * @return
 */
/*bool check_DH(train_drive& drive,rs::train_physics const& tp){
  auto brake_end = drive.phases_.front().back();
  auto end_dist = drive.phases_.back().back().dist_;
  drive.phases_.back().pop_back();
  while(brake_end.speed_>pt.v_min_&&brake_end.dist_<pt.distance_){
    auto interval = get_interval(brake_end.dist_);
    auto deaccel =
tp.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
    auto next_brake =
soro::runtime::rk4::brake_over_distance(brake_end.speed_,deaccel,interval.length());
    if(next_brake.speed_<pt.v_min_) next_brake =
soro::runtime::rk4::brake(brake_end.speed_,pt.v_min_,deaccel);
    brake_end.dist_+=next_brake.dist_;
    brake_end.time_+=next_brake.time_;
    brake_end.speed_ = next_brake.speed_;
    drive.phases_.front().push_back(next_brake);
    if(brake_end.time_+(end_dist-brake_end.dist_)/brake_end.speed_>=pt.e_time_){
      if(brake_end.dist_!=pt.distance_){
        soro::runtime::train_state cruise_begin(brake_end);
        soro::runtime::train_state
cruise_end(si::time(brake_end.time_+(end_dist-brake_end.dist_)/brake_end.speed_),pt.distance_,brake_end.speed_);
        drive.phases_.push_back({cruise_begin,cruise_end});
      }
      else drive.phase_types_.pop_back();
      return true;
    }
  }
  utls::sassert(brake_end.dist_<pt.distance_,"Reaching TPE at distance {} after
e_time {} is impossible, even with max braking",pt.distance_,pt.e_time_); auto
slowest_drive = slowest_drive(); train_drive slow_drive(slowest_drive);
  if(has_halt(slowest_drive)){
    mach was;
    return true;
  }
  auto offset_time = brake_end.time_;
  for(int i=1;i<slow_drive.phases_.front().size();++i){
    slow_drive.phases_.front()[i].time_+=offset_time;
    drive.phases_.front().push_back(slow_drive.phases_.front()[i]);
    auto intersection_state = get_intersection;
    if(get_time_after_cruise(slow_drive.phases_.front()[i],intersection_state)+remaining_time()>=pt.v_min_){
      drive.phase_types_.push_back(acceleration);
      drive.phases_.push_back({slow_drive.phases_.front()[i],intersection_state});
      drive.phases_.push_back(remaning_accel_states);
      return true;
    }
  }
  throw std::logic_error("Slowest drive is still too fast for TPE at distance
"+pt.distance_.to_string()+", e_time_ "+pt.e_time_.to_string()+" and start speed
"+drive.phases_.front().front().speed_.to_string());
}*/

/**
 *
 * @param drive
 * @return
 */
bool check_DA(train_drive& drive) {
  throw std::logic_error("not implemented DA");
}
/**
 * tries to alter a ride thats solely an acceleration phase to take more time.
 * It may throw an exception if it finds that a long enough ride doesnt exist
 * @param drive the ride
 * @param pt the point that is the destination
 * @return true iff it finds a slow enough drive
 */
// Nicht ganz fertig
bool check_A(train_drive& drive) {
  bool state_changed = false;
  auto& phase = drive.phases_.front();
  for (int i = phase.size() - 2; i >= 0; --i) {
    auto state = phase[i];
    if (state.speed_ <= pt.v_min_) {
      state =
          state.speed_ == pt.v_min_
              ? state
              : find_state_with_speed(pt.v_min_, drive.phases_.front(), true);
      // Nein, falsch, hier muss ein slowest drive gemacht werden
      if (state.time_ + (pt.distance_ - state.dist_) / pt.v_min_ < pt.e_time_)
        throw std::logic_error("TPE cant be driven.");
      state_changed = true;
    }
    if (i == 0 || state.time_ + (pt.distance_ - state.dist_) / state.speed_ >=
                      pt.e_time_) {
      drive.phases_.front().erase(drive.phases_.front().begin() + i + 1,
                                  drive.phases_.front().end());
      train_state end_state(
          state.time_ + (pt.distance_ - state.dist_) / state.speed_,
          pt.distance_, state.speed_);
      auto cruise_phase = {state, end_state};
      if (state_changed) drive.phases_.front().push_back(state);
      if (drive.phases_.front().size() == 1) {
        drive.phases_.pop_back();
        drive.phase_types_.pop_back();
      }
      drive.phases_.push_back(cruise_phase);
      drive.phase_types_.push_back(cruising);
      return state.time_ + (pt.distance_ - state.dist_) / state.speed_ >=
             pt.e_time_;
    }
  }
  throw std::logic_error("for in check_A didnt return a value");
}
/**
 * this method tries to change the ride to reach pt after its earliest time.
 * It introduces a new brake phase and possibly an acceleration afterwards.
 * Either this finds a fitting ride or it throws an exception.
 * @param drive the ride
 * @param pt the train_path_envelope point that is the destination
 * @param tp the train physics
 * @return true if the method finds a ride thats long enough
 */
/*bool check_H(train_drive& drive,soro::train_path_envelope::tpe_point const&
pt,soro::rs::train_physics const& tp){ auto beginning_state =
drive.phases_.front().front(); auto interval =
get_interval(beginning_state.dist_); auto slowest_drive = slowest_drive(); auto
[brake_accel_switch,index] = find_switch(slowest_drive);
  soro::vector<soro::runtime::train_state> brake_states = {beginning_state};
  while(beginning_state.speed_!=pt.v_min_){
    beginning_state =
soro::runtime::rk4::brake_over_distance(beginning_state.speed_,tp.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length()),interval.end_distance());
    if(beginning_state.dist_>brake_accel_switch.dist_) beginning_state =
brake_accel_switch; brake_states.push_back(beginning_state);
    if((pt.distance_-beginning_state.dist_)/beginning_state.speed_+beginning_state.time_>=pt.e_time_){
      soro::vector<soro::runtime::train_state> hold_phase =
{beginning_state,{(pt.distance_-beginning_state.dist_)/beginning_state.speed_+beginning_state.time_,beginning_state.dist_,beginning_state.speed_}};
      brake_states.push_back(beginning_state);
      drive.phases_ = {brake_states,hold_phase};
      drive.phase_types_ = {braking,cruising};
      return true;
    }
  }
  if(beginning_state == brake_accel_switch) {
    if(slowest_drive.back().time()<pt.e_time_) throw std::logic_error("This
shouldnt happen"); drive.phase_types_ = {braking,acceleration}; drive.phases_ =
{brake_states,{slowest_drive.begin()+index,slowest_drive.end()}}; return true;
  }
  throw std::logic_error("TPE not drivable");
}*/
/**
 * this methode is called when the entire ride is only one brake phase
 * this method will always throw an exception. This is because if the train always brakes along the ride and still doesnt take long enough, then we are too fast at the beginning. This situation should never happen
 * @param drive the ride
 * @return nothing
 */
bool check_D(train_drive& drive) {
  if (!(drive.phases_.size() == 1 && drive.phase_types_.size() == 1 &&
        drive.phase_types_.front() == braking)) {
    std::for_each(drive.phase_types_.begin(), drive.phase_types_.end(),
                  [](phase_type type) { std::cout << type << std::endl; });
    throw std::logic_error("Wrong method");
  }
  if (drive.phases_.back().back().time_ >= pt.e_time_)
    throw std::logic_error(
        "Train is already slow enough, no use in calling this");
  throw std::logic_error(
      "Train takes too short even with constant braking. This should never happen.");
}
}// namespace increase_time