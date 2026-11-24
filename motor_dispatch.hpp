#pragma once

#include <cstddef>

static constexpr std::size_t kMaxMotors = 32;

// Abstract base; the concrete Motor class must implement these.
class Motor {
public:
    virtual ~Motor() = default;
    virtual std::size_t dependency_count() const noexcept = 0;
    virtual Motor*      dependency(std::size_t i) const noexcept = 0;
    virtual void        calculate() noexcept = 0;
};

// Flat array of motor pointers in topological order.
// Lives in whichever memory region the caller allocates it in.
struct MotorDispatchList {
    Motor*      motors[kMaxMotors];
    std::size_t count{0};
};

// Build a topological dispatch order using Kahn's algorithm.
// all_motors[0..n-1] are all known motors.
// On success  : list is filled in order and true is returned.
// On cycle    : list is left unchanged and false is returned.
//               The caller should keep the current active buffer and
//               report an error.
bool build_dispatch_order(MotorDispatchList* list,
                          Motor* const*      all_motors,
                          std::size_t        n) noexcept;
