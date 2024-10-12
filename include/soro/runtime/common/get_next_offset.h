#pragma once
#include "increase_time.h"
namespace increase_time{
using namespace soro;
int search_indizes(train_drive const& drive,vector<phase_type> const& types);
int standard_next_offset(train_drive const& drive);
}// namespace increase_time
