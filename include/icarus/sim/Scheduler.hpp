#pragma once

#include <cstddef>
#include <vector>

namespace icarus {

/**
 * @brief Execution scheduler for components.
 *
 * Determines the order in which components are executed during Step().
 * Supports topological sorting based on signal dependencies.
 */
class Scheduler {
  public:
    virtual ~Scheduler() = default;

    /**
     * @brief Get execution order for components.
     *
     * @return Vector of component indices in execution order
     */
    [[nodiscard]] virtual std::vector<std::size_t> GetExecutionOrder() const = 0;
};

} // namespace icarus
