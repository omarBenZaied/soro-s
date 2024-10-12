#pragma once
#include "soro/runtime/common/get_next_offset.h"
namespace increase_time{

int search_indizes(train_drive const& drive,vector<phase_type> const& types){
  utls::sassert(!types.empty()&&types.size()<=drive.phase_types_.size(),"types is either too long or empty");
  vector<phase_type>last_seen(drive.phase_types_.end()-types.size(),drive.phase_types_.end());
  for(int i = drive.phase_types_.size()-types.size();i>0;--i){
    if(last_seen==types) return i;
    last_seen.pop_back();
    last_seen.insert(last_seen.begin(),drive.phase_types_[i-1]);
  }
  if(last_seen==types) return 0;
  return -1;
}
int standard_next_offset(train_drive const& drive){
  auto index = search_indizes(drive,{acceleration,cruising,braking});
  if(index!=-1) return index;
  index = search_indizes(drive,{acceleration,braking});
  if(index!=-1) return index;
  index = search_indizes(drive,{acceleration,cruising});
  if(index!=-1) return index;
  index = search_indizes(drive,{cruising,braking});
  if(index!=-1) return index;
  return drive.phase_types_.size()-3;
}


} // namespace increase_time
