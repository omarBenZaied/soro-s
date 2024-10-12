#pragma once
#include "soro/runtime/common/increase_time.h"
#include "soro/runtime/common/phase_checkers.h"
#include "soro/timetable/timetable.h"
#include "soro/runtime/strategy/shortest_travel_time.h"
#include "soro/runtime/common/signal_time.h"
#include "soro/runtime/common/tpe_respecting_travel.h"
#include "tpe_simulation_utls.h"
namespace increase_time{
using namespace soro;
using namespace soro::runtime;
using namespace soro::train_path_envelope;
using namespace tpe_simulation;
int next_index(train_drive const& drive,int const& start_index,phase_checker const& checker);
void AHD_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,tt::train const& t);
void AHA_check(train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,tt::train const& t);
void HDH_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,tt::train const& t);
void test_check_function(soro::vector<tt::train> const& trains,infra::infrastructure const& infra,infra::type_set const& record_types,int function_to_use);
}// namespace increase_time