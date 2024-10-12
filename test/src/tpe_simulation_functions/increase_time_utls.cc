#pragma once
#include "doctest/doctest.h"
#include "soro/runtime/common/increase_time.h"
#include "soro/runtime/common/phase_checkers.h"
#include "soro/timetable/timetable.h"
#include "soro/runtime/strategy/shortest_travel_time.h"
#include "soro/runtime/common/signal_time.h"
#include "soro/runtime/common/tpe_respecting_travel.h"
#include "test/tpe_runtime/tpe_simulation_utls.h"
namespace increase_time{
using namespace soro;
using namespace soro::runtime;
using namespace soro::train_path_envelope;
using namespace tpe_simulation;
int next_index(train_drive const& drive,int const& start_index,phase_checker const& checker){
  if(drive.phases_.empty()) return -1;
  for(int i=start_index;i<drive.phases_.size()-1;++i){
    auto t1 = drive.phase_types_[i];
    auto t2 = drive.phase_types_[i+1];
    auto t3 = i+2<drive.phases_.size()?drive.phase_types_[i+2]:increase_time::invalid;
    if(checker(t1,t2,t3)) return i;
  }
  return -1;
}
void AHD_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,tt::train const& t){
  if(drive.phases_.empty()) return;
  increase_time::set_pt(point);
  int index = next_index(drive,0,increase_time::AHD_checker);
  while(index!=-1){
    bool finished = increase_time::check_AHD(drive,index);
    CHECK_GE(drive.phases_.size(),1);
    check_drive(drive,0);
    utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_AHD returned wrong result {}",finished);
    check_drivable(drive,intr_points,t,0);
    if(finished) return;
    index= next_index(drive,0,increase_time::AHD_checker);
  }
}
void AHA_check(train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,tt::train const& t){
  if(drive.phases_.empty()) return;
  set_pt(point);
  set_intervals(intr_points);
  int index = next_index(drive,0,increase_time::AHA_checker);
  while(index!=-1){
    bool finished = check_AHA(drive,t.physics_,index);
    check_drive(drive,index);
    utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_AHA returned wrong result {}",finished);
    check_drivable(drive,intr_points,t,index);
    if(finished) return;
    index=next_index(drive,0,AHA_checker);
  }
}
void HDH_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,tt::train const& t){
  if(drive.phases_.empty()) return;
  set_pt(point);
  set_intervals(intr_points);
  int index = next_index(drive,0,HDH_checker);
  while(index!=-1){
    bool finished = increase_time::check_HDH(drive,t.physics_,index);
    check_drive(drive,index);
    utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_AHA returned wrong result {}",finished);
    check_drivable(drive,intr_points,t,index);
    if(finished) return;
    index=next_index(drive,0,increase_time::HDH_checker);
  }
}
void DA_check(increase_time::train_drive& drive,vector<interval_point> const& intr_points,tt::train const& t) {
  if(drive.phases_.empty()) return;
  int index  = next_index(drive,0,DA_checker);
  if(index==-1) return;
  std::cout<<"DA tested"<<std::endl;
  drive.erase_elements(0,index);
  auto size = drive.phases_.size();
  drive.erase_elements(2,size-2);
  auto state = drive.phases_.back().back();
  tpe_point point(state.dist_,state.time_*1.1,state.time_*1.1,si::speed::zero(),si::speed::infinity());
  set_pt(point);
  set_intervals(intr_points);
  bool finished = check_DA(drive, t.physics_);
  check_drive(drive, 0);
  utl::verify(finished == (drive.phases_.back().back().time_ >= point.e_time_),
              "check_DA returned wrong result {}", finished);
  check_drivable(drive, intr_points, t, 0);
}

void HA_check(increase_time::train_drive& drive,vector<interval_point> const& intr_points,tt::train const& t){
  int index = next_index(drive,0,HA_checker);
  while(index!=-1){
    std::cout<<"HA tested"<<std::endl;
    train_drive copy_drive;
    auto phase_it = drive.phases_.begin()+index;
    auto type_it = drive.phase_types_.begin()+index;
    copy_drive.phases_.insert(copy_drive.phases_.begin(),phase_it,phase_it+2);
    copy_drive.phase_types_.insert(copy_drive.phase_types_.begin(),type_it,type_it+2);
    copy_drive.start_state_ = copy_drive.phases_.front().front();
    auto state = copy_drive.phases_.back().back();
    tpe_point point(state.dist_,state.time_*1.1,state.time_*1.1,si::speed::zero(),si::speed::infinity());
    set_pt(point);
    set_intervals(intr_points);
    bool finished = check_HA(copy_drive,t.physics_);
    check_drive(copy_drive,0);
    utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_HA returned wrong result {}",finished);
    check_drivable(copy_drive,intr_points,t,0);
    index = next_index(drive,index+1,HA_checker);
  }
}

void test_check_function(soro::vector<tt::train> const& trains,infra::infrastructure const& infra,infra::type_set const& record_types,int function_to_use) {
  shortest_travel_time shortest_travel_time;
  signal_time const signal_time;
  auto const ARRIVAL_FACTOR = 1.1;
  for (auto const& t : trains) {
    auto tpe_points = get_tpe_points(t, infra, record_types);
    tpe_simulation::merge_duplicate_tpe_points(tpe_points);
    auto intervals = tpe_simulation::split_intervals(
        get_intervals(t, record_types, infra), tpe_points, t.physics_);
    train_state current;
    current.time_ = si::time(t.start_time_.count());
    current.dist_ = si::length::zero();
    current.speed_ = t.start_speed_;
    increase_time::train_drive drive;
    drive.start_state_ = current;
    tt::train::trip const trip(tt::train::trip::id{0}, t.id_, ZERO<absolute_time>);
    auto point_index = intervals.begin().length().is_zero() ? 0 : 1;
    for (auto const& interval : intervals) {
      auto [delta, delta_drive] = shortest_travel_time.create_drive(
          current, nullptr, interval, t, trip, signal_time);
      drive += delta_drive;
      current += delta;
      if (interval.end_distance() == tpe_points[point_index].distance_) {
        tpe_points[point_index].e_time_ = tpe_points[point_index].e_time_*ARRIVAL_FACTOR;
        tpe_points[point_index].l_time_ = std::max(tpe_points[point_index].l_time_,tpe_points[point_index].e_time_);
        if(function_to_use==0) AHD_check(drive,tpe_points[point_index],intervals.p_,t);
        if(function_to_use==1) AHA_check(drive,tpe_points[point_index],intervals.p_,t);
        if(function_to_use==2) HDH_check(drive,tpe_points[point_index],intervals.p_,t);
        if(function_to_use==3) DA_check(drive,intervals.p_,t);
        if(function_to_use==4) HA_check(drive,intervals.p_,t);
        drive.erase_elements(0, drive.phases_.size());
        drive.start_state_ = current;
        ++point_index;
      }
    }
  }
}
}// namespace increase_time
