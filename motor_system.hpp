#pragma once

#include "double_buffer.hpp"
#include "motor_dispatch.hpp"

// Concrete Motor implementation with dependency tracking.
class MotorAxis : public Motor {
public:
    static constexpr std::size_t kMaxDeps = 4;

    std::size_t dependency_count() const noexcept override { return dep_count_; }
    Motor*      dependency(std::size_t i) const noexcept override { return deps_[i]; }
    void        calculate() noexcept override { /* ... */ }

    // Replace this axis's entire dependency list (called from main loop only).
    void set_deps(Motor* const* deps, std::size_t n) noexcept {
        dep_count_ = (n <= kMaxDeps) ? n : kMaxDeps;
        for (std::size_t i = 0; i < dep_count_; ++i) deps_[i] = deps[i];
    }

    void clear_deps() noexcept { dep_count_ = 0; }

private:
    Motor*      deps_[kMaxDeps] = {};
    std::size_t dep_count_      = 0;
};


// Manages the motor list and the double-buffered dispatch order.
class MotorSystem {
public:
    // Register a motor. Must be called before any ISR cycle starts.
    bool add_motor(MotorAxis* m) noexcept {
        if (motor_count_ >= kMaxMotors) return false;
        all_motors_[motor_count_++] = m;
        return true;
    }

    // Build the initial dispatch order. Call once after all motors are added.
    bool init() noexcept {
        return rebuild_spare_and_commit();
    }

    // Change the dependency of `follower` to depend on `leader`
    // (pass nullptr to clear the dependency).
    // Returns true on success, false if the change would create a cycle.
    // On false the graph and the active buffer are left unchanged.
    bool set_dependency(MotorAxis* follower, Motor* leader) noexcept {
        // Save current deps so we can roll back on cycle.
        Motor*      saved_deps[MotorAxis::kMaxDeps];
        std::size_t saved_count = follower->dependency_count();
        for (std::size_t i = 0; i < saved_count; ++i)
            saved_deps[i] = follower->dependency(i);

        // Apply the tentative change.
        if (leader) follower->set_deps(&leader, 1);
        else        follower->clear_deps();

        // Try to build a valid order into the spare buffer.
        if (!rebuild_spare_and_commit()) {
            // Cycle detected — roll back.
            follower->set_deps(saved_deps, saved_count);
            return false;
        }
        return true;
    }

    // ISR side: call once at the start of each 1 ms cycle.
    void isr_cycle() noexcept {
        const MotorDispatchList* list = dispatch_db_.active();  // read once
        for (std::size_t i = 0; i < list->count; ++i)
            list->motors[i]->calculate();
    }

private:
    bool rebuild_spare_and_commit() noexcept {
        if (!build_dispatch_order(dispatch_db_.spare(), all_motors_, motor_count_))
            return false;  // cycle detected, spare is untouched
        dispatch_db_.commit();
        return true;
    }

    MotorAxis*              all_motors_[kMaxMotors] = {};
    std::size_t             motor_count_            = 0;

    // Both buffers live here (or place them in a specific section via linker).
    MotorDispatchList       buf_a_;
    MotorDispatchList       buf_b_;
    DoubleBuffer<MotorDispatchList> dispatch_db_{&buf_a_, &buf_b_};
};
