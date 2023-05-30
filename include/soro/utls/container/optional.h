#pragma once

#include <cassert>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>

#include "cista/serialization.h"

#include "soro/base/time.h"

namespace soro::utls {

template <typename T>
  requires cista::is_pointer_v<T>
constexpr T get_default_invalid() {
  return T{nullptr};
}

template <std::integral T>
constexpr T get_default_invalid() {
  return std::numeric_limits<T>::max();
}

template <typename T>
  requires soro::soro_time<T>
constexpr T get_default_invalid() {
  return T::max();
}

template <typename T>
struct optional {
  constexpr static T INVALID = get_default_invalid<T>();

  optional() noexcept = default;
  optional(std::nullopt_t) {}
  explicit optional(T const val) noexcept : val_{val} {}  // NOLINT

  optional(optional const&) = default;
  optional& operator=(optional const&) = default;

  optional(optional&&) noexcept = default;
  optional& operator=(optional&&) noexcept = default;

  static_assert(
      std::is_trivially_destructible_v<std::decay_t<T>>,
      "if T would be a more complicated type we'd need to ensure destruction");
  ~optional() = default;

  auto operator<=>(optional const&) const = default;

  T const* operator->() const noexcept {
    assert(has_value());
    return &val_;
  }

  T* operator->() noexcept {
    assert(has_value());
    return &val_;
  }

  T const& operator*() const& noexcept { return value(); }

  T& operator*() & noexcept { return value(); }

  T const&& operator*() const&& noexcept { return value(); }

  T& operator*() && noexcept { return value(); }

  explicit operator bool() const noexcept { return has_value(); }

  bool has_value() const noexcept { return val_ != INVALID; }

  T& value() & {
    assert(has_value());
    return val_;
  }

  T const& value() const& {
    assert(has_value());
    return val_;
  }

  T& value() && {
    assert(has_value());
    return val_;
  }

  T const& value() const&& {
    assert(has_value());
    return val_;
  }

  template <typename U>
  T value_or(U&& default_value) const& noexcept {
    if (has_value()) {
      return val_;
    } else {
      return static_cast<T>(default_value);
    }
  }

  template <typename U>
  T value_or(U&& default_value) && noexcept {
    return value_or(default_value);
  }

  void reset() noexcept { val_ = INVALID; }

  template <typename Fn>
  void execute_if(Fn&& fn) const noexcept {
    if (has_value()) {
      fn(value());
    }
  }

  template <typename ThenFunction>
  auto and_then(ThenFunction&& then_function) const noexcept {
    using return_t =
        std::remove_cvref_t<std::invoke_result_t<ThenFunction, T const&>>;

    if (has_value()) {
      return then_function(value());
    } else {
      return return_t{};
    }
  }

  template <typename TransformFunction>
  auto transform(TransformFunction&& transform_function) const noexcept {
    using return_t =
        optional<std::invoke_result_t<TransformFunction, T const&>>;

    if (has_value()) {
      return return_t{transform_function(this->value())};
    } else {
      return return_t{std::nullopt};
    }
  }

  template <typename ElseFunction>
  optional or_else(ElseFunction&& else_function) const& noexcept {
    return has_value() ? *this : std::forward<ElseFunction>(else_function)();
  }

  template <typename ElseFunction>
  optional or_else(ElseFunction&& else_function) && noexcept {
    return or_else(else_function);
  }

  void swap(optional& other) noexcept {
    if (has_value() && !other.has_value()) {
      other = this->val_;
      this->reset();
    } else if (!has_value() && other.has_value()) {
      this->val_ = other.value();
      other.reset();
    } else if (has_value() && other.has_value()) {
      auto const tmp = this->value();
      this->val_ = other.value();
      other = tmp;
    }
  }

#if !defined(SERIALIZE)
private:
#endif
  T val_{INVALID};
};

template <typename T>
optional(T) -> optional<T>;

template <typename T>
  requires std::is_integral_v<T> || std::is_pointer_v<T>
optional<T> make_optional(T&& value) {
  return optional<T>(std::forward<T>(value));
}

template <typename Ctx, typename T, T INVALID>
inline void serialize(Ctx& context, optional<T> const* opt,
                      cista::offset_t const offset) {
  using cista::serialize;

  // otherwise offsetof is not working
  static_assert(std::is_standard_layout_v<optional<T>>);

  serialize(context, &opt->val_,
            offset + static_cast<cista::offset_t>(
                         offsetof(std::remove_pointer_t<decltype(opt)>, val_)));
}

template <typename Ctx, typename T, T INVALID>
inline void deserialize(Ctx const& context, optional<T>* opt) {
  using cista::deserialize;
  deserialize(context, &opt->val_);
}

template <typename T, T INVALID>
cista::hash_t type_hash(optional<T> const&, cista::hash_t h,
                        std::map<cista::hash_t, unsigned>& done) noexcept {
  h = cista::hash_combine(h, cista::hash("soro::utls::optional"));
  h = cista::type_hash(T{}, h, done);
  return h;
}

}  // namespace soro::utls