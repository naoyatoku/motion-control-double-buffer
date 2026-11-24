#pragma once

#include <atomic>
#include <cstddef>

// Generic lock-free double buffer for use between a main-loop and an ISR.
//
// The ISR calls active() once at the start of each cycle and uses that
// pointer for the whole cycle. The main loop builds into spare(), then
// calls commit(). The index change is an acquire/release pair, so no
// interrupt masking is needed.
//
// Memory placement is entirely the caller's responsibility: allocate both
// buffers in the desired section (TCM, system RAM, …) and pass them in.
// This makes the same switching logic reusable across memory types.
//
// Lifecycle rule: after commit(), wait at least one full ISR cycle before
// touching spare() again, because the ISR may still be reading what was
// just swapped out.
//
// Usage — plain struct (e.g. MotorDispatchList):
//
//   MotorDispatchList buf_a, buf_b;
//   DoubleBuffer<MotorDispatchList> db(&buf_a, &buf_b);
//
//   // main loop, when dependency changes:
//   build_dispatch_order(db.spare(), motors, n);
//   db.commit();
//
//   // ISR:
//   auto* list = db.active();   // read once
//   for (size_t i = 0; i < list->count; ++i) list->motors[i]->calculate();
//
// Usage — placement new (e.g. PulseGeneratorSet):
//
//   // Initial construction (both buffers must be live objects):
//   new(db.spare()) PulseGeneratorSet(initial_params);
//   db.commit();
//   new(db.spare()) PulseGeneratorSet(initial_params);
//
//   // On new command:
//   db.spare()->~PulseGeneratorSet();
//   new(db.spare()) PulseGeneratorSet(new_params);
//   db.commit();
//   // --- wait one ISR cycle ---
//   // spare() now points to the old active; destroy it before next use:
//   db.spare()->~PulseGeneratorSet();

template <typename T>
class DoubleBuffer {
public:
    DoubleBuffer(T* buf_a, T* buf_b) noexcept
        : bufs_{buf_a, buf_b}, active_idx_{0u} {}

    // ISR side: call once per cycle, use the returned pointer for the whole
    // cycle. The acquire load pairs with the release store in commit().
    T* active() const noexcept {
        return bufs_[active_idx_.load(std::memory_order_acquire)];
    }

    // Main-loop side: returns the buffer that is not currently active.
    T* spare() const noexcept {
        return bufs_[spare_idx()];
    }

    // Main-loop side: atomically makes the spare buffer the active one.
    // All writes to spare() must be complete before this call.
    void commit() noexcept {
        active_idx_.store(spare_idx(), std::memory_order_release);
    }

private:
    unsigned spare_idx() const noexcept {
        return active_idx_.load(std::memory_order_relaxed) ^ 1u;
    }

    T* bufs_[2];
    std::atomic<unsigned> active_idx_;
};
