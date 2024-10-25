
#pragma once
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/common/train_safety.h"
#include "soro/timetable/train.h"
namespace soro::tpe_simulation {
using namespace soro::train_path_envelope;
void merge_duplicate_tpe_points(tpe_points& points);
runtime::intervals split_intervals(runtime::intervals const& intervals, tpe_points const& pts,rs::train_physics const& tp);
std::tuple<runtime::train_state,increase_time::train_drive> get_end_state(runtime::train_state current_state,tpe_point const& pt, runtime::interval& interval,
                          runtime::train_safety* train_safety,tt::train const& train,tt::train::trip const& trip,si::time const& prev_e_time);
vector<runtime::interval_point> fix_intervals(runtime::interval& interval,tpe_point const& point,rs::train_physics const& tp);
vector<runtime::train_state> tpe_respecting_simulation(
    tpe_points& tpe_points, runtime::train_state const& initial,runtime::train_safety* train_safety,
    tt::train const& train, tt::train::trip const& trip,
    infra::infrastructure const& infra,infra::type_set const& record_types);
} // namespace soro::tpe_simulation