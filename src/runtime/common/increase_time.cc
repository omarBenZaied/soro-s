#include "soro/runtime/common/increase_time.h"
//#include "soro/runtime/common/train_path_envelope.h"
//#include "soro/runtime/physics/rk4/detail/rk4_step.h"
//#include "soro/runtime/physics/rk4/brake.h"
#include "soro/rolling_stock/train_physics.h"
#include "soro/runtime/physics/rk4/detail/delta_t.h"
#pragma once
/**
 * main method to increase the time of a ride by changing the speed profile
 * @param drive the ride
 */
/*void increase_time(train_drive& drive,soro::train_path_envelope::tpe_point const& pt,soro::rs::train_physics const& tp){
  auto AHD_checker = [](phase_type t1,phase_type t2,phase_type t3){return t1==acceleration&&(t2==braking||t3==braking);};
  auto AHA_checker = [](phase_type t1,phase_type t2,phase_type t3){return t1==acceleration&&t2==cruising;};
  auto HDH_checker = [](phase_type t1,phase_type t2,phase_type t3){return t1==cruising&&t2==braking;};
  auto DHA_checker = [](phase_type t1,phase_type t2,phase_type t3){return t1==braking&&t2==cruising&&t3==acceleration;};
  while(drive.phases_.size()>=3){
    auto phases_it = drive.phases_.begin();
    auto types_it = drive.phase_types_.begin();
    while(types_it<drive.phase_types_.end()-1){
      auto t1 = *types_it;
      auto t2 = *(types_it+1);
      auto t3 = types_it+2<drive.phase_types_.end()?*(types_it+2):invalid;
      bool case_found = false;
      if(AHD_checker(t1,t2,t3)){
        auto finished = check_AHD(drive,phases_it,types_it);
        if(finished) return;
        case_found = true;
      }
      if(!case_found&&AHA_checker(t1,t2,t3)){
        auto finished = check_AHA(drive,types_it,phases_it);
        if(finished) return;
        case_found = true;
      }
      if(!case_found&&HDH_checker(t1,t2,t3)){
        auto finished = check_HDH(drive,types_it,phases_it);
        if(finished) return;
        case_found = true;
      }
      if(!case_found&&DHA_checker(t1,t2,t3)){
        auto finished = check_DHA(drive,types_it,phases_it);
        if(finished) return;
        case_found = true;
      }
      ++types_it;
      ++phases_it;
    }
  }
  auto HA_checker = [](phase_type t1,phase_type t2){return t1==cruising&&t2==acceleration;};
  auto DA_checker = [](phase_type t1,phase_type t2){return t1==braking&&t2==acceleration;};
  auto DH_checker = [](phase_type t1,phase_type t2){return t1==braking&&t2==cruising;};
  while(drive.phases_.size()==2){
    auto p_t_1 = drive.phase_types_.front();
    auto p_t_2 = drive.phase_types_.back();
    if(AHD_checker(p_t_1,p_t_2,invalid)){
     auto finished = check_AHD(drive);
     if(finished) return;
    }
    if(AHA_checker(p_t_1,p_t_2,invalid)){
      auto finished = check_AHD(drive);
      if(finished) return;
    }
    if(HDH_checker(p_t_1,p_t_2,invalid)){
      auto finished = check_AHD(drive);
      if(finished) return;
    }
    if(HA_checker(p_t_1,p_t_2)){
      auto finished = check_AHD(drive);
      if(finished) return;
    }
    if(DA_checker(p_t_1,p_t_2)){
      auto finished = check_AHD(drive);
      if(finished) return;
    }
    if(DH_checker(p_t_1,p_t_2)){
      auto finished = check_AHD(drive);
      if(finished) return;
    }
  }
  if(drive.phase_types_.front()==acceleration){
    auto finished = check_A(drive);
    if(finished) return;
  }
  if(drive.phase_types_.front()==cruising){
    auto finished = check_H(drive);
    if(finished) return;
  }
  if(drive.phase_types_.front()==braking) check_D(drive);
}*/

/*bool check_AHD(train_drive& drive, decltype(drive.phases_.begin()) phase_it,decltype(drive.phase_types_.begin()) type_it,soro::train_path_envelope::tpe_point const& pt){
  auto t_real = drive.phases_.back().back().time_;
  bool second_is_braking = *(type_it+1)==braking;
  if(!(*type_it==acceleration&&(second_is_braking||*(type_it+2)==braking))) throw std::logic_error("AHD got wrong types");
  auto brake_phase = second_is_braking?*(phase_it+1):*(phase_it+2);
  auto accel_phase = *phase_it;
  auto min_cruise_v = std::max(accel_phase.front().speed_,brake_phase.back().speed_);
  auto cruise_v = brake_phase.front().speed_;
  auto max_reduction = cruise_v-min_cruise_v;
  auto [distance,t_alt] = find_distance_and_t_alt(max_reduction);
  if(max_reduction<=-(distance/pt.e_time_-t_real+t_alt)+cruise_v){
    auto result = calculate_results(max_reduction);
    int delete_offset = second_is_braking?1:2;
    drive.phases_.erase(phase_it,phase_it+delete_offset);
    drive.phase_types_.erase(type_it,type_it+delete_offset);
    drive.phase_types_.insert(type_it,result.phase_types);
    drive.phases_.insert(phase_it,result.phases_);
    return max_reduction==-(distance/pt.e_time_-t_real+t_alt)+cruise_v;
  }
  int brake_index = brake_phase.size()-2;
  soro::si::length end_distance;
  for(int i=accel_phase.size()-2;i>=0;--i){
   auto start_distance = accel_phase[i].dist_;
   soro::si::time end_time;
   for(;brake_index>=0;--brake_index){
     if(brake_phase[brake_index].speed_==accel_phase[i].speed_){
       end_distance = brake_phase[brake_index].dist_;
       end_time = brake_phase[brake_index].time_;
       break;
     }
     if(brake_phase[brake_index].speed_<accel_phase[i].speed_){
       auto decceleration = ;
       auto time = -(brake_phase[brake_index+1].speed_-accel_phase[i].speed_)/decceleration;
       end_distance = brake_phase[brake_index+1].speed_*time+0.5*decceleration*time.pow<2>()+brake_phase[brake_index+1].dist_;
       end_time = brake_phase[brake_index+1].time_+time;
       break;
     }
   }
   if((end_distance-start_distance)/accel_phase[i].speed_+accel_phase[i].time_-end_time>=pt.e_time_-t_real){
     calculate_results
     return true;
   }
  }
  throw std::logic_error("check_AHD should not get here");
}*/

/*bool check_AHA(train_drive& drive,decltype(drive.phases_.begin()) phase_it,decltype(drive.phase_types_.begin()) type_it,soro::train_path_envelope::tpe_point const& pt,soro::rs::train_physics const& tp){
  auto t_real = drive.phases_.back().back().time_;
  auto cruise = *(phase_it+1);
  soro::runtime::train_state state = cruise.back();
  auto first_acceleration_index = phase_it->size()-2;
  bool result_found = false;
  while(state.speed_!=phase_it->front().speed_){
    auto delta = soro::runtime::rk4::rk4_step(state.speed_,soro::runtime::rk4::delta_t,get_slope(state.dist_),tp);
    if(state.speed_-delta.speed_<phase_it->front().speed_){
      auto speed_dif = phase_it->front().speed_-state.speed_;
      // hier muss ich die intersection finden
      delta = speed_dif/delta.speed_*delta;
    }
    soro::si::length start_distance;
    soro::si::time start_time;
    for(;first_acceleration_index>=0;--first_acceleration_index){
      auto accel_state = phase_it->at(first_acceleration_index);
      if(accel_state.speed_>state.speed_) continue;
      //TODO: hier muss ich auch eine intersection zwischen geschwindigkeiten finden
      //schau hier nochmal drüber
      auto accel = find_acceleration();
      auto time = soro::si::time{(state.speed_ - accel_state.speed_).val_};
      start_distance = accel_state.dist_+accel_state.speed_*time+0.5*accel*time.pow<2>();
      start_time = accel_state.time_+time;
      break;
    }
    if((state.dist_-start_distance)/state.speed_+cruise.back().time_-state.time_-cruise.back().time_+start_time>=pt.e_time_-t_real){
      result_found = true;
      break;
    }
  }
  calculate_result();
  return result_found;
}*/
/**
 * tries to change part of a ride to make the ride take long enough.
 * It only inspects a part of the ride that has a H-D-H Phase-chain
 * @param drive the ride
 * @return true iff the methode finds an alteration that increases the time enough
 */
/*bool check_HDH(train_drive& drive,decltype(drive.phases_.begin()) phase_it,decltype(drive.phase_types_.begin()) types_it,soro::rs::train_physics const& tp,int offset){
  auto initial = phase_it->front();
  auto interval = get_interval(initial.dist_);
  auto brake_end_speed = (phase_it+1)->back().speed_;
  soro::vector<soro::runtime::train_state> brake = {initial};
  while(initial.speed_!=brake_end_speed){
    auto deccel = tp.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
    initial = brake_over_distance_with_target(initial,deccel,brake_end_speed);
    brake.push_back(initial);
  }
  auto end_of_new_cruise_state = phase_it+2>=drive.phases_.end()||*(types_it+2)!=cruising ? (phase_it+1)->back():(phase_it+2)->back();
  auto current_pt_time = drive.phases_.back().back().time_;
  if(initial.time_+(end_of_new_cruise_state.dist_-initial.dist_)/initial.speed_-end_of_new_cruise_state.time_<=pt.e_time_-current_pt_time){
    auto to_erase = phase_it+2>=drive.phases_.end()||*(types_it+2)!=cruising ? 2:3;
    soro::runtime::train_state end_cruise;
    end_cruise.dist_ = end_of_new_cruise_state.dist_;
    end_cruise.speed_ = initial.speed_;
    end_cruise.time_ = initial.time_ +(end_of_new_cruise_state.dist_-initial.dist_)/initial.speed_;
    soro::vector<soro::runtime::train_state> cruise_phase = {initial,end_cruise};
    drive.phases_.erase(phase_it,phase_it+to_erase);
    drive.phase_types_.erase(types_it,types_it+to_erase);
    phase_it = drive.phases_.begin()+offset;
    types_it = drive.phase_types_.begin()+offset;
    drive.phase_types_.insert(types_it,braking);
    drive.phase_types_.insert(drive.phase_types_.begin()+offset+1,cruising);
    drive.phases_.insert(phase_it,brake);
    drive.phases_.insert(drive.phases_.begin()+offset+1,cruise_phase);
    return initial.time_+(end_of_new_cruise_state.dist_-initial.dist_)/initial.speed_-end_of_new_cruise_state.time_==pt.e_time_-current_pt_time;
  }
  auto start_dist = find_start_distance(drive,phase_it,types_it);

}*/
/**
 *
 * @param drive
 * @return
 */
bool check_DHA(train_drive& drive){
return true;
}
/**
 *
 * @param drive
 * @return
 */
bool check_HA(train_drive& drive){
return true;
}
/**
 *
 * @param drive
 * @return
 */
bool check_DH(train_drive& drive){
  return true;
}

/**
 *
 * @param drive
 * @return
 */
bool check_DA(train_drive& drive){
 return true;
}
/**
 * tries to alter a ride thats solely an acceleration phase to take more time.
 * It may throw an exception if it finds that a long enough ride doesnt exist
 * @param drive the ride
 * @param pt the point that is the destination
 * @return true iff it finds a slow enough drive
 */
/*bool check_A(train_drive& drive,soro::train_path_envelope::tpe_point const& pt){
  for(int i = drive.phases_.front().size()-2;i>=0;--i){
    auto state = drive.phases_.front()[i];
    if(state.speed_<=pt.v_min_){
     state = find_state_with_speed(pt.v_min_,state,drive.phases_.front()[i+1]);
     if(state.time_+(pt.distance_-state.dist_)/pt.v_min_<pt.e_time_) throw std::logic_error("TPE cant be driven.");
    }
    if(state.time_+(pt.distance_-state.dist_)/state.speed_>=pt.e_time_||i==0) {
      drive.phases_.front().erase(drive.phases_.front().begin()+i+1,drive.phases_.front().end());
      soro::runtime::train_state end_state(state.time_+(pt.distance_-state.dist_)/state.speed_,pt.distance_,state.speed_);
      auto hold_phase = {state,end_state};
      if(drive.phases_.front().size()==1){
        drive.phases_.pop_back();
        drive.phase_types_.pop_back();
      }
      drive.phases_.push_back(hold_phase);
      drive.phase_types_.push_back(cruising);
      return state.time_+(pt.distance_-state.dist_)/state.speed_>=pt.e_time_;
    }
  }
  // This cant happen but sure CLION
  throw std::logic_error("for in check_A didnt return a value");
}*/
/**
 * this method tries to change the ride to reach pt after its earliest time.
 * It introduces a new brake phase and possibly an acceleration afterwards.
 * Either this finds a fitting ride or it throws an exception.
 * @param drive the ride
 * @param pt the train_path_envelope point that is the destination
 * @param tp the train physics
 * @return true if the method finds a ride thats long enough
 */
/*bool check_H(train_drive& drive,soro::train_path_envelope::tpe_point const& pt,soro::rs::train_physics const& tp){
  auto beginning_state = drive.phases_.front().front();
  auto interval = get_interval(beginning_state.dist_);
  auto slowest_drive = slowest_drive();
  auto [brake_accel_switch,index] = find_switch(slowest_drive);
  soro::vector<soro::runtime::train_state> brake_states = {beginning_state};
  while(beginning_state.speed_!=pt.v_min_){
    beginning_state = soro::runtime::rk4::brake_over_distance(beginning_state.speed_,tp.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length()),interval.end_distance());
    if(beginning_state.dist_>brake_accel_switch.dist_) beginning_state = brake_accel_switch;
    brake_states.push_back(beginning_state);
    if((pt.distance_-beginning_state.dist_)/beginning_state.speed_+beginning_state.time_>=pt.e_time_){
      soro::vector<soro::runtime::train_state> hold_phase = {beginning_state,{(pt.distance_-beginning_state.dist_)/beginning_state.speed_+beginning_state.time_,beginning_state.dist_,beginning_state.speed_}};
      brake_states.push_back(beginning_state);
      drive.phases_ = {brake_states,hold_phase};
      drive.phase_types_ = {braking,cruising};
      return true;
    }
  }
  if(beginning_state == brake_accel_switch) {
    if(slowest_drive.back().time()<pt.e_time_) throw std::logic_error("This shouldnt happen");
    drive.phase_types_ = {braking,acceleration};
    drive.phases_ = {brake_states,{slowest_drive.begin()+index,slowest_drive.end()}};
    return true;
  }
  throw std::logic_error("TPE not drivable");
}*/
/**
 * this methode is called when the entire ride is only one brake phase
 * this method will always throw an exception. This is because if the train always brakes along the ride
 * and still doesnt take long enough, then we are too fast at the beginning.
 * This situation should never happen
 * @param drive the ride
 * @return nothing
 */
bool check_D(train_drive& drive){
  if(!(drive.phases_.size()==1&&drive.phase_types_.size()==1&&drive.phase_types_.front()==braking)){
    std::for_each(drive.phase_types_.begin(),drive.phase_types_.end(),[](phase_type type){std::cout<<type<<std::endl;});
    throw std::logic_error("Wrong method");
  }
  throw std::logic_error("Train takes too short even with constant brakin. This should never happen.");
}