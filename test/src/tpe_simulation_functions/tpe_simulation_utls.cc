#pragma once
#include "doctest/doctest.h"
#include "test/tpe_runtime/tpe_simulation_utls.h"

namespace soro::tpe_simulation{
using namespace soro;
using namespace soro::train_path_envelope;
using namespace soro::runtime;
tpe_points get_tpe_points(tt::train const& t, infra::infrastructure const& infra,
                          infra::type_set const& record_types){
  auto timestamps = rk4::runtime_calculation(t,infra,record_types,runtime::use_surcharge::no);
  tpe_points pts;
  for(auto e:timestamps.times_) {
    tpe_point point(e.dist_, si::time(e.arrival_.count()),
                    si::time(e.departure_.count()), si::speed::zero(),
                    si::speed::infinity());
    pts.push_back(point);
  }
  return pts;
}

void check_drive(increase_time::train_drive const& drive,int const& offset){
  if(offset>=drive.phases_.size()) return;
  auto unique_it = std::adjacent_find(drive.phase_types_.begin()+offset,drive.phase_types_.end());
  if(unique_it!=drive.phase_types_.end()) utl::fail("2 neighboring phases {} at offsets {} and {}",*unique_it,unique_it-drive.phase_types_.begin(),unique_it-drive.phase_types_.begin()+1);
  train_state predecessor;
  for(int i=offset;i<drive.phases_.size();++i){
    if(!std::is_sorted(drive.phases_[i].begin(),drive.phases_[i].end(),[](train_state const& st1,train_state const& st2){return st1.dist_<st2.dist_;})){
      utl::fail("states arent ordered in distance");
    }
    if(drive.phases_[i].end()!=std::adjacent_find(drive.phases_[i].begin(),drive.phases_[i].end(),[](train_state const& st1,train_state const& st2){return st1.dist_==st2.dist_;})){
      utl::fail("states with same distance");
    }
    predecessor = offset>0?drive.phases_[offset-1].back():drive.start_state_;
    CHECK_EQ(drive.phases_[offset].front().dist_,predecessor.dist_);
    CHECK_EQ(drive.phases_[offset].front().speed_,predecessor.speed_);
    CHECK_EQ(drive.phases_[offset].front().time_,predecessor.time_);
    switch (drive.phase_types_[i]){
      case increase_time::acceleration:
        if(!std::is_sorted(drive.phases_[i].begin(),drive.phases_[i].end(),
                            [](train_state const& state1,train_state const& state2){
                              return state1.speed_<state2.speed_;
                            })) throw std::logic_error("speed in acceleration is not ascending");
        break;
      case increase_time::cruising:
        CHECK_EQ(drive.phases_[i].size(),2);
        CHECK_EQ(drive.phases_[i].front().speed_,drive.phases_[i].back().speed_);
        if(!drive.phases_[i].front().speed_.is_zero())CHECK_EQ(drive.phases_[i].back().time_-drive.phases_[i].front().time_,increase_time::get_cruise_time(drive.phases_[i].front(),drive.phases_[i].back()));
        break;
      case increase_time::braking:
        if(!std::is_sorted(drive.phases_[i].begin(),drive.phases_[i].end(),
                            [](train_state const& state1,train_state const& state2){
                              return state1.speed_>state2.speed_;
                            })) throw std::logic_error("brake isnt descending in speed");
        break;
      default:throw std::logic_error("Invalid phase type detected");
    }
  }
}

void check_acceleration_possible(vector<train_state> const& accel,vector<interval_point> const& intr_point,tt::train const& t){
  for(auto const& state: accel){
    auto it = utls::find_if(intr_point,[state](interval_point const& point){return point.distance_>=state.dist_;});
    utls::sassert(it!=intr_point.end(),"state has too high distance");
    if(it+1==intr_point.end()&&it->distance_==state.dist_) continue;
    interval current_interval  = it->distance_>state.dist_?interval(&*(it-1),&*it):interval(&*it,&*(it+1));
    auto tractive_force = t.physics_.tractive_force(state.speed_);
    auto resistive_force = t.physics_.resistive_force(state.speed_,current_interval.slope());
    if(resistive_force.is_negative()) std::cout<<"resistive force is negative"<<std::endl;
    utls::sassert(tractive_force>resistive_force.abs(),"acceleration not possible");
  }
}
void check_braking_possible(vector<train_state> const& brake,vector<interval_point> const& intr_point,tt::train const& t){
  for(auto const& state:brake){
    if(state.dist_ == intr_point.back().distance_)continue;
    auto it = utls::find_if(intr_point,[state](interval_point const& point){return point.distance_>=state.dist_;});
    utls::sassert(it!=intr_point.end(),"state has too high distance");
    interval current_interval  = it->distance_>state.dist_?interval(&*(it-1),&*it):interval(&*it,&*(it+1));
    auto deaccel = t.physics_.braking_deaccel(current_interval.infra_limit(),current_interval.bwp_limit(),current_interval.brake_path_length());
    utls::sassert(deaccel.is_negative(),"positive deaccel");
  }
}
void check_cruise_possible(vector<train_state> const& cruise,vector<interval_point> const& intr_point,tt::train const& t){
  auto& start_state = cruise.front();
  auto& end_state = cruise.back();
  auto speed = start_state.speed_;
  auto start_it = utls::find_if(intr_point,[start_state](interval_point const& point){return point.distance_>=start_state.dist_;});
  auto end_it = utls::find_if(intr_point,[end_state](interval_point const& point){return point.distance_>=end_state.dist_;});
  start_it = start_it->distance_ == start_state.dist_ ? start_it : start_it-1;
  while(start_it<end_it-1){
    auto tractive_force = t.physics_.tractive_force(speed);
    auto resistive_force = t.physics_.resistive_force(speed,start_it->slope_);
    utls::sassert(tractive_force>=resistive_force,"tractive_force too weak, cruising is impossible");
    ++start_it;
  }
}

void check_drivable(increase_time::train_drive const& drive,vector<interval_point> const& intr_point,tt::train const& t, int const& offset){
  for(int i=offset;i<drive.phases_.size();++i){
    auto phase = drive.phases_[i];
    switch (drive.phase_types_[i]){
      case increase_time::acceleration:
        check_acceleration_possible(phase,intr_point,t);
        break;
      case increase_time::braking:
        check_braking_possible(phase,intr_point,t);
        break;
      case increase_time::cruising:
        check_cruise_possible(phase,intr_point,t);
        break;
      default:
        throw std::logic_error("invalid type detected");
    }
  }
}

}