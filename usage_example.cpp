#include "double_buffer.hpp"
#include "motor_dispatch.hpp"

// ---------------------------------------------------------------------------
// Example 1: dispatch order  (plain Motor* arrays, no placement new needed)
// ---------------------------------------------------------------------------

// Place both buffers in the desired memory region via linker attributes.
// Here they are in TCM for fast ISR access.
static Motor* dispatch_order_a[kMaxMotors] __attribute__((section(".tcm_data")));
static Motor* dispatch_order_b[kMaxMotors] __attribute__((section(".tcm_data")));

static DoubleBuffer<Motor*> dispatch_db(dispatch_order_a, dispatch_order_b);

// All motors, and how many. Every motor is always in the order, so the order
// has motor_count entries. motor_count changes only before the ISR starts.
static Motor* all_motors[kMaxMotors];
static std::size_t motor_count = 0;

// Called from the 1 ms ISR:
void isr_1ms() {
    Motor* const* order = dispatch_db.active();  // read once
    for (std::size_t i = 0; i < motor_count; ++i) {
        order[i]->calculate();
    }
}

// Called from the main loop when a dependency changes:
void on_dependency_changed() {
    if (!build_dispatch_order(motor_count, all_motors, dispatch_db.spare())) {
        // Circular dependency — keep the current active buffer, report error.
        return;
    }
    dispatch_db.commit();
    // Wait one ISR cycle before calling spare() again (e.g. check a cycle flag).
}


// ---------------------------------------------------------------------------
// Example 2: PulseGeneratorSet  (constructed via placement new into the buffer)
// ---------------------------------------------------------------------------

// Forward declaration of the actual pulse generator set type.
struct PulseGeneratorSet {
    // ... pulse generator objects live here ...
    explicit PulseGeneratorSet(/* params */) { /* construct generators */ }
    ~PulseGeneratorSet() { /* destroy generators */ }
};

// Both buffers live in system memory (required by the pulse hardware).
// alignas ensures placement new is well-defined.
static alignas(PulseGeneratorSet) unsigned char pgen_raw_a[sizeof(PulseGeneratorSet)]
    __attribute__((section(".sysram")));
static alignas(PulseGeneratorSet) unsigned char pgen_raw_b[sizeof(PulseGeneratorSet)]
    __attribute__((section(".sysram")));

// Construct both objects before handing memory to the double buffer.
static PulseGeneratorSet* pgen_ptr_a = new(pgen_raw_a) PulseGeneratorSet(/* initial params */);
static PulseGeneratorSet* pgen_ptr_b = new(pgen_raw_b) PulseGeneratorSet(/* initial params */);

static DoubleBuffer<PulseGeneratorSet> pgen_db(pgen_ptr_a, pgen_ptr_b);

// Called from the main loop when a new motion command arrives:
void on_new_command(/* new_params */) {
    // Destroy the old spare object and construct the new one in its place.
    pgen_db.spare()->~PulseGeneratorSet();
    new(pgen_db.spare()) PulseGeneratorSet(/* new_params */);

    pgen_db.commit();

    // After one full ISR cycle the previous active buffer is safe to touch.
    // The next call to on_new_command() will destroy it via spare()->~T()
    // above, so no explicit cleanup is needed here.
}
