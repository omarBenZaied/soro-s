#pragma once

#include "soro/base/time.h"

#include "soro/infrastructure/graph/element.h"

namespace soro::runtime {

struct event : times {
  auto operator<=>(event const& e) const {
    return element_<=>e.element_;
  }

  infra::element::ptr element_{nullptr};
  si::length dist_{si::length::zero()};
};

using EventCB = std::function<void(event const&)>;

}  // namespace soro::runtime