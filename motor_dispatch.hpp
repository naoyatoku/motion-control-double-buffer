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

// Build a topological dispatch order using Kahn's algorithm (topological_sort.hpp).
// all_motors[0..n-1] are all known motors. The order is written to order[0..n-1]
// (a plain Motor* array; the count is n, kept by the caller).
// Every dependency must be in all_motors (checked with assert; with NDEBUG a
// missing one makes it return false, the same as a cycle).
// The same dependency listed twice is fine (it is not mistaken for a cycle).
// On success  : order is filled and true is returned.
// On cycle    : false is returned. order may be partly overwritten, so pass the
//               spare buffer and do not commit it; the active buffer keeps the
//               current order. The caller should report an error.
bool build_dispatch_order(std::size_t   n,
                          Motor* const  all_motors[],
                          Motor*        order[]) noexcept;
