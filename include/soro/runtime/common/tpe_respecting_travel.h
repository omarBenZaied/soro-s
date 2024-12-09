
#pragma once
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/common/train_safety.h"
#include "soro/timetable/train.h"
#include "soro/runtime/common/train_state.h"
#include "soro/runtime/common/interval.h"
#include "soro/runtime/common/increase_time.h"
namespace soro::tpe_simulation {
using namespace soro::train_path_envelope;
void merge_duplicate_tpe_points(tpe_points& points);
runtime::intervals split_intervals(runtime::intervals const& intervals, tpe_points const& pts,rs::train_physics const& tp);
std::tuple<runtime::train_state,increase_time::train_drive> get_end_state(runtime::train_state current_state,tpe_point const& pt, runtime::interval& interval,
                          runtime::train_safety* train_safety,tt::train const& train,tt::train::trip const& trip);
vector<runtime::interval_point> fix_intervals(runtime::interval& interval,tpe_point const& point,rs::train_physics const& tp);
std::tuple<vector<runtime::train_state>,increase_time::train_drive> tpe_respecting_simulation(
    tpe_points& tpe_points, runtime::train_state const& initial,runtime::train_safety* train_safety,
    tt::train const& train, tt::train::trip const& trip,
    infra::infrastructure const& infra,infra::type_set const& record_types);
vector<runtime::train_state> slowest_drive(runtime::train_state initial, tpe_point const& pt, runtime::interval interval,rs::train_physics const& tp);
void reset_duration();
std::chrono::microseconds get_duration();
int get_count();
} // namespace soro::tpe_simulation