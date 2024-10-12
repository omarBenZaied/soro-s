#pragma once
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/common/get_intervals.h"

#include "soro/timetable/timetable.h"

#include "soro/runtime/common/use_surcharge.h"
#include "soro/runtime/rk4_runtime.h"
namespace soro::tpe_simulation{
train_path_envelope::tpe_points get_tpe_points(tt::train const& t, infra::infrastructure const& infra,
                          infra::type_set const& record_types);
void check_drive(increase_time::train_drive const& drive,int const& offset);
void check_drivable(increase_time::train_drive const& drive,vector<runtime::interval_point> const& intr_point,tt::train const& t, int const& offset);
} // soro::tpe_simulation