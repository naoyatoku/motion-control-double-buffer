#include "motor_dispatch.hpp"

bool build_dispatch_order(MotorDispatchList* list,
                          Motor* const*      all_motors,
                          std::size_t        n) noexcept {
    if (n > kMaxMotors) return false;

    // in_degree[i] = number of predecessors of all_motors[i]
    std::size_t in_degree[kMaxMotors] = {};

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t d = 0; d < all_motors[i]->dependency_count(); ++d) {
            Motor* dep = all_motors[i]->dependency(d);
            for (std::size_t k = 0; k < n; ++k) {
                if (all_motors[k] == dep) { ++in_degree[i]; break; }
            }
        }
    }

    // Kahn's algorithm: start with all zero-in-degree motors
    Motor*      queue[kMaxMotors];
    std::size_t q_head = 0, q_tail = 0;

    for (std::size_t i = 0; i < n; ++i) {
        if (in_degree[i] == 0) queue[q_tail++] = all_motors[i];
    }

    Motor*      order[kMaxMotors];
    std::size_t order_count = 0;

    while (q_head < q_tail) {
        Motor* m = queue[q_head++];
        order[order_count++] = m;

        // Reduce in-degree of every motor that depends on m
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t d = 0; d < all_motors[i]->dependency_count(); ++d) {
                if (all_motors[i]->dependency(d) == m) {
                    if (--in_degree[i] == 0) queue[q_tail++] = all_motors[i];
                    break;
                }
            }
        }
    }

    if (order_count != n) return false;  // circular dependency detected

    for (std::size_t i = 0; i < n; ++i) list->motors[i] = order[i];
    list->count = n;
    return true;
}
