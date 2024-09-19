//
// Created by Omar Ben Zaied on 15.08.2024.
//
#pragma once
#include "soro/infrastructure/infrastructure.h"
#include "soro/runtime/common/get_intervals.h"
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/strategy/shortest_travel_time.h"
#include "soro/utls/sassert.h"
#include "soro/runtime/physics/rk4/brake.h"
#include "soro/runtime/physics/rk4/accelerate.h"
#include "soro/runtime/physics/rk4/detail/brake_accel_intersection.h"
#include "soro/utls/std_wrapper/any_of.h"
//#include "soro-s/src/runtime/common/get_intervals.cc"
namespace soro::tpe_simulation {
using namespace soro::runtime;
using namespace soro::train_path_envelope;
using namespace soro;

/**
 * splits intervals at tpe_points.
 * @param intervals the intervals of the journey
 * @param pts an ordered list of points
 * @param tp train physics
 * @return the new intervals object
 */
intervals split_intervals(intervals const& intervals, tpe_points const& pts,rs::train_physics const& tp) {
  if(pts.empty()) return intervals;
  struct intervals result;
  auto intr_points = intervals.p_;
  intr_points.reserve(intr_points.size()+pts.size());
  auto it = intr_points.begin();
  auto it_offset = 0;
  utls::sassert(it->distance_<=pts.front().distance_,"TPE_Points before the ride even begins.");
  utls::sassert(intr_points.back().distance_>=pts.back().distance_,"TPE_Points happen after end.");
  for (auto const& point : pts) {
    while (it->distance_ < point.distance_) {
      ++it;
      ++it_offset;
    }
    if (it->distance_ != point.distance_) {
      auto intr_point = *(it - 1);
      interval_point new_point(intr_point);
      auto records_it = utls::find_if((it-1)->records_,[point](record const& r){return r.dist_>=point.distance_;});
      auto records_to_move = soro::vector<record>{records_it,(it-1)->records_.end()};
      (it-1)->records_.erase(records_it,(it-1)->records_.end());
      new_point.distance_ = point.distance_;
      new_point.records_ = records_to_move;
      intr_points.insert(it, new_point);
      it = intr_points.begin()+(++it_offset);
    }
  }
  for(auto i=intr_points.size()-1;i>0;--i){
    fix_short_interval(intr_points[i-1],intr_points[i],tp);
  }
  result.p_ = intr_points;
  return result;
}
/**
 * merges tpe_points that are at the same distance.
 * This is done by deleting all points that are at the same distance
 * and replacing them with a point at that distance with values that enforce both points.
 * This methode requires points to be ordered by distance
 * @param points the ordered tpe_points
 */
void merge_duplicate_tpe_points(tpe_points& points){
  auto it = points.begin();
  auto it_offset = 0;
  while(it<(points.end()-1)){
    if(it->distance_==(it+1)->distance_){
      auto it2 = it+1;
      auto new_pt = tpe_point(it->distance_,std::max(it->e_time_,it2->e_time_),std::min(it->l_time_,it2->l_time_),std::max(it->v_min_,it2->v_min_),std::min(it->v_max_,it2->v_max_));
      points.erase(it,it+2);
      it = points.begin()+it_offset;
      points.insert(it,new_pt);
      it = points.begin()+it_offset;
    }
    else {
      ++it_offset;
      ++it;
    }
  }
}
/**
 * finds the intersection between a braking process and an accaleration process if tere is one.
 * You need to make sure that one exists.
 * @param brake_states the states of the brake process. They should all be at the interval points
 * @param accel_states the states of the acceleration process. They should all be at interval points
 * @param interval the starting interval of the brake process
 * @param tp the train physics
 * @return the intersection of the processes as well as the index of the last brake_state before the intersection.
 * If no intersection is found, it returns a zero state as well as -1.
 */
std::tuple<train_state,int> find_intersection(soro::vector<train_state> brake_states, soro::vector<train_state> accel_states,interval interval,rs::train_physics const& tp){
  int i=0;
  for(;i<brake_states.size()&&brake_states[i].dist_!=accel_states.back().dist_;++i){}
  if(i==brake_states.size()) return {train_state{},-1};
  int j=0;
  for(;j+i<brake_states.size();++j){
    if(brake_states[i+j].speed_==accel_states[accel_states.size()-1-j].speed_) return {brake_states[i+j],i+j};
    if(j+i+1<brake_states.size()&&brake_states[i+j].speed_>accel_states[accel_states.size()-1-j].speed_&&brake_states[i+j+1].speed_<accel_states[accel_states.size()-2-j].speed_){
      break;
    }
  }
  if(j+i==brake_states.size()) return {train_state{},-1};
  auto distance = brake_states[i+j].dist_;
  while(interval.start_distance()!=distance) ++interval;
  return {b_a_intersection::brake_accel_intersection(brake_states[i+j],accel_states[accel_states.size()-1-j],interval,tp),i+j};
}

bool check_intersection(train_state brake_state, train_state accel_state,train_state corresponding_accel_state,train_state corresponding_brake_state){
  if(brake_state.dist_<accel_state.dist_) return false;
  return corresponding_brake_state.speed_>=accel_state.speed_&&
  corresponding_accel_state.speed_>=brake_state.speed_;
}

soro::vector<train_state> slowest_drive(train_state initial, tpe_point const& pt, interval interval,rs::train_physics const& tp){
  auto start_distance = initial.dist_;
  auto start_speed = initial.speed_;
  train_state final_state;
  final_state.speed_ = pt.v_min_;
  final_state.dist_ = pt.distance_;
  auto intr_copy = interval;
  // ich muss schauen, dass das eine Kopie erstellt
  auto end_interval = interval;
  while(end_interval.end_distance()!=pt.distance_) ++end_interval;
  soro::vector<train_state> brake_states = {initial};
  soro::vector<train_state> accel_states = {final_state};
  auto corresponding_accel_index = 0;
  auto corresponding_brake_index = 0;
  bool lines_overlap = false;
  while(!check_intersection(initial,final_state,accel_states[corresponding_accel_index],brake_states[corresponding_brake_index])&&(initial.speed_.is_positive()||final_state.speed_.is_positive())){
    if(!lines_overlap) corresponding_accel_index = accel_states.size()-1;
    while(initial.dist_<pt.distance_&&initial.speed_>accel_states[corresponding_accel_index].speed_){
      auto deaccel = tp.braking_deaccel(interval.infra_limit(), interval.bwp_limit(),
                                        interval.brake_path_length());
      initial+= soro::runtime::rk4::brake_over_distance(initial.speed_,deaccel,interval.end_distance()-interval.start_distance());
      brake_states.push_back(initial);
      ++interval;
      if(lines_overlap){
        --corresponding_accel_index;
      }
      if(!lines_overlap&&initial.dist_>=final_state.dist_){
        lines_overlap=true;
      }
    }
    if(initial.dist_==pt.distance_&&initial.speed_>pt.v_max_) throw std::logic_error("Braking in slowest_drive still results in speed greater then v_max_");
    if(initial.dist_==pt.distance_&&initial.speed_>pt.v_min_) return brake_states;
    //Das hier funktioniert nicht
    if(!lines_overlap) corresponding_brake_index=brake_states.size()-1;
    while(final_state.dist_>start_distance&&final_state.speed_>brake_states[corresponding_brake_index].speed_){
      final_state = soro::runtime::rk4::accelerate_backwards(final_state,end_interval,tp);
      accel_states.push_back(final_state);
      --end_interval;
      if(lines_overlap){
        --corresponding_brake_index;
      }
      if(!lines_overlap&&final_state.dist_<=initial.dist_) {
        lines_overlap = true;
      }
    }
    if(final_state.dist_==start_distance&&final_state.speed_>start_speed) throw std::logic_error("Cant accelerate from initial speed to minimum speed");
    if(final_state.dist_==start_distance&&final_state.speed_==start_speed) {
      std::reverse(accel_states.begin(),accel_states.end());
      return accel_states;
    }
  }
  if(!check_intersection(initial,final_state,accel_states[corresponding_accel_index],brake_states[corresponding_brake_index])){
    return brake_states;
    //return cruise_to_halt(brake_states,final_state,++end_interval);
  }
  auto [state,index] = find_intersection(brake_states,accel_states,intr_copy,tp);
  utls::sassert(index!=-1,"find_intersection found no intersection even though there should be one");
  //FInde nach richtiger stelle in accel_states
  accel_states.push_back(state);
  std::reverse(accel_states.begin(),accel_states.end());
  brake_states.erase(brake_states.begin()+index+1,brake_states.end());
  brake_states.insert(brake_states.end(),accel_states.begin(),accel_states.end());
  auto binary_pred = [](train_state s1,train_state s2){return s1.dist_==s2.dist_&&s1.speed_==s2.speed_;};
  auto new_end = std::unique(brake_states.begin(),brake_states.end(),binary_pred);
  return {brake_states.begin(),new_end};
}

bool check_slowest_drive(train_state state, tpe_point const& pt, interval interval, rs::train_physics const& tp){
  auto states = slowest_drive(state,pt,interval,tp);
  bool has_halt = utls::detail::any_of(states,[](train_state e){return e.speed_.is_zero();});
  return has_halt||states.back().time_>=pt.e_time_;
}
si::speed get_max_allowed_speed(si::length distance,tpe_point const& pt,interval interval,rs::train_physics const& tp,si::time min_allowed_time){
  train_state state;
  state.dist_ = pt.distance_;
  state.speed_ = pt.v_min_;
  utls::sassert(state.dist_ == interval.end_distance(),"get_max_allowed_speed expected {}, but got {}",pt.distance_,interval.end_distance());
  while(state.speed_.is_positive()&&state.dist_>distance){
    state = soro::runtime::rk4::accelerate_backwards(state,interval,tp);
    if(state.dist_==interval.start_distance())--interval;
  }
  utls::sassert(state.dist_>=distance,"state got backwards simulated too far get_max_allowed_speed");
  if(state.dist_==distance) {
    if(-state.time_<min_allowed_time) throw std::logic_error("TPE is not drivable.");
    return state.speed_;
  }
  while(state.dist_!=distance){
    auto deaccel = tp.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
    state = soro::runtime::rk4::brake_backwards(state,deaccel,interval.start_distance(),interval.speed_limit(tp));
  }
  return state.speed_;
}
si::speed get_min_allowed_speed(si::length const& distance, interval interval,train_state& state,rs::train_physics const& tp){
  while(state.dist_!=distance&&state.speed_.is_positive()){
    state = soro::runtime::rk4::accelerate_backwards(state,interval,tp);
    --interval;
  }
  return state.speed_;
}
si::speed get_possible_max_speed(si::length const& distance, interval interval,train_state& state,rs::train_physics const& tp){
  while(state.dist_!=distance) {
    state = soro::runtime::rk4::brake_backwards(state,tp.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length()),interval.start_distance(),std::min(interval.speed_limit(tp),(interval-1).speed_limit(tp)));
    if(interval.start_distance()!=distance) state.speed_ = std::min(state.speed_,(interval-1).speed_limit(tp));
    --interval;
  }
  return state.speed_;
}
void fix_tpe_speeds(tpe_points& points,interval interval,rs::train_physics const& tp, train_state const& start_state){
  while(interval.end_distance()!=points.back().distance_) ++interval;
  train_state end_state;
  for(int i=points.size()-1;i>0;--i){
    end_state.speed_ = points[i].v_min_;
    end_state.dist_ = points[i].distance_;
    points[i-1].v_min_ = std::max(points[i-1].v_min_,get_min_allowed_speed(points[i-1].distance_,interval,end_state,tp));
    end_state.speed_ = points[i].v_max_;
    end_state.dist_ = points[i].distance_;
    points[i-1].v_max_ = std::min(points[i-1].v_max_,get_possible_max_speed(points[i-1].distance_,interval,end_state,tp));
    //Überprüfe, ob ich interval manuell ändern muss
  }
  end_state.speed_ = points.front().v_min_;
  end_state.dist_ = points.front().distance_;
  auto min_speed = get_min_allowed_speed(start_state.dist_,interval,end_state,tp);
  if(start_state.speed_<min_speed) throw std::logic_error("speed of start state is too low, tpe is not drivable");
  auto max_speed = get_possible_max_speed(start_state.dist_,interval,end_state,tp);
  if(start_state.speed_>max_speed) throw std::logic_error("speed of start is too fast, tpe not drivable");
}

soro::vector<interval_point> fix_intervals(interval interval,tpe_point const& point,rs::train_physics const& tp){
  soro::vector<interval_point> new_intervals;
  while(interval.end_distance()!=point.distance_){
    new_intervals.push_back(*interval.p1_);
    ++interval;
  }
  while(interval.end_distance()==point.distance_){
    new_intervals.push_back(*interval.p1_);
    ++interval;
  }
  new_intervals.push_back(*interval.p1_);
  if(new_intervals[new_intervals.size()-1].limit_>point.v_max_){
    new_intervals[new_intervals.size()-1].limit_ = point.v_max_;
    for(auto i=new_intervals.size()-1;i>0;--i) {
      fix_short_interval(new_intervals[i-1],new_intervals[i],tp);
    }
  }
  return new_intervals;
}

train_state get_end_state(train_state current_state,tpe_point const& pt, interval& interval,
                    train_safety* train_safety,soro::tt::train const& train,soro::tt::train::trip const& trip){
  signal_time signal_time;
  signal_time.time_ = ZERO<absolute_time>;
  shortest_travel_time shortest_travel_time;
  auto new_intervals = fix_intervals(interval,pt,train.physics_);
  struct interval fixed_interval = {new_intervals.data(),new_intervals.data()+1};
  while(current_state.dist_<pt.distance_){
    /*auto p = interval.p2_;
    if(interval.end_distance()==pt.distance_&&pt.v_max_<p->limit_){
      auto new_interval_point = *interval.p2_;
      new_interval_point.limit_ = pt.v_max_;
      interval.p2_ = &new_interval_point;
    }*/
    current_state += shortest_travel_time.drive(
        current_state, train_safety, fixed_interval, train, trip, signal_time);
    current_state.dist_ = fixed_interval.end_distance();
    //if(interval.end_distance()==pt.distance_) interval.p2_ = p;
    ++interval;
    ++fixed_interval;
    utls::sassert(
        current_state.time_ <= pt.l_time_,
        "TPE-Point at distance {} with latest time {} can't be achieved",
        pt.distance_, pt.l_time_);
  }
  return current_state;
}
soro::vector<train_state> tpe_respecting_simulation(
    tpe_points& tpe_points, train_state const& initial, train_safety* train_safety,
    soro::tt::train const& train, soro::tt::train::trip const& trip,
    soro::infra::infrastructure infra) {
  std::sort(tpe_points.begin(), tpe_points.end(),
            [](tpe_point e1, tpe_point e2) { return e1 < e2; });
  merge_duplicate_tpe_points(tpe_points);
  auto route_intervals =
      get_intervals(train, soro::infra::type_set::all(), infra);
  intervals intervals = split_intervals(route_intervals, tpe_points,train.physics_);
  //beginning_distance = intervals.begin().start_distance();
  soro::vector<train_state> result;
  result.reserve(tpe_points.size()+1);
  result.push_back(initial);
  auto interval = intervals.begin();
  fix_tpe_speeds(tpe_points,interval,train.physics_,initial);
  auto current_state = initial;
  for (int i = 0; i < tpe_points.size() ; ++i) {
    auto end_point = tpe_points[i];
    current_state = get_end_state(current_state,end_point,interval,train_safety,train,trip);
    utls::sassert(current_state.dist_==end_point.distance_,"Distance of current_state isnt equal to distance of tpe_point. I dont know how that happened, but it sure did.");
    //das ist ein interessanter fall der nur auftreten kann zwischen dem initialzustand und dem ersten tpe-punkt
    //if(current_state.speed_<end_point.v_min_) throw std::logic_error("Minimum speed of "+std::to_string(end_point.v_min_.val_)+" at distance "+std::to_string(end_point.distance_.val_)+" cant be driven");
    if(i+1<tpe_points.size()&&!check_slowest_drive(current_state,tpe_points[i+1],interval,train.physics_)){
      auto max_allowed_speed = get_max_allowed_speed(end_point.distance_,tpe_points[i+1],interval-1,train.physics_,tpe_points[i+1].l_time_-end_point.e_time_);
      end_point.v_max_ = max_allowed_speed;
      //TODO: überprüfe hier, ob interval richtig ist.
      current_state = get_end_state(result.back(),end_point,interval,train_safety,train,trip);
    }
    if(current_state.time_<end_point.e_time_){
      //current_state = increase_time(result.back());
    }
    result.push_back(current_state);
  }
  return result;
}

}  // namespace tpe_simulation
