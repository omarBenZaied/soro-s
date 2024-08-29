#pragma once

#include "soro/timetable/train.h"

#include "soro/runtime/common/interval.h"

namespace soro::runtime {

intervals get_intervals(tt::train const& train,
                        infra::type_set const& record_types,
                        infra::infrastructure const& infra);

void fix_short_interval(interval_point& p1, interval_point const& p2,
                        rs::train_physics const& tp);

}  // namespace soro::runtime
