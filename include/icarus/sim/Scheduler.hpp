#pragma once

/**
 * @file Scheduler.hpp
 * @brief Group-based execution scheduler for components
 *
 * Part of Phase 4.0.7: Configuration Infrastructure.
 * Implements multi-rate execution with frame divisors.
 */

#include <icarus/io/MissionLogger.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Group-based execution scheduler
 *
 * Manages component execution order based on:
 * - Group priority (which group runs first)
 * - Member priority (which component runs first within a group)
 * - Frame divisors (multi-rate: group runs when frame_count % divisor == 0)
 *
 * Each group has a rate_hz. The simulation runs at the fastest rate.
 * Slower groups have divisor > 1 and run less frequently.
 */
class Scheduler {
  public:
    /**
     * @brief Configure scheduler from config
     *
     * @param config Scheduler configuration with groups
     * @param sim_rate_hz Global simulation rate (fastest group rate)
     */
    void Configure(const SchedulerConfig &config, double sim_rate_hz) {
        config_ = config;
        sim_rate_hz_ = sim_rate_hz;

        // Sort groups by priority
        std::sort(config_.groups.begin(), config_.groups.end(),
                  [](const SchedulerGroupConfig &a, const SchedulerGroupConfig &b) {
                      return a.priority < b.priority;
                  });

        // Sort members within each group by priority
        for (auto &group : config_.groups) {
            std::sort(
                group.members.begin(), group.members.end(),
                [](const GroupMember &a, const GroupMember &b) { return a.priority < b.priority; });
        }

        // Compute frame divisors
        config_.ComputeFrameDivisors(sim_rate_hz);

        // Note: LogExecutionOrder() is called by Simulator after Configure
        // with proper logger from config
    }

    /**
     * @brief Get groups that should execute on this frame
     *
     * @param frame_count Current frame number (0-indexed)
     * @param current_phase Current flight phase (-1 to ignore phase filtering)
     * @return Vector of group names that should execute
     */
    [[nodiscard]] std::vector<std::string> GetGroupsForFrame(int frame_count,
                                                             int32_t current_phase = -1) const {
        std::vector<std::string> active_groups;
        for (const auto &group : config_.groups) {
            // Check frame divisor
            auto it = config_.group_frame_divisors.find(group.name);
            if (it != config_.group_frame_divisors.end()) {
                int divisor = it->second;
                if (frame_count % divisor != 0) {
                    continue; // Not this frame
                }
            }

            // Check phase gating (if phase is provided)
            if (current_phase >= 0 && !group.IsActiveInPhase(current_phase)) {
                continue; // Not active in this phase
            }

            active_groups.push_back(group.name);
        }
        return active_groups;
    }

    /**
     * @brief Get members for a group in execution order
     *
     * @param group_name Name of the group
     * @return Vector of component names in execution order
     */
    [[nodiscard]] std::vector<std::string> GetMembersForGroup(const std::string &group_name) const {
        std::vector<std::string> members;
        for (const auto &group : config_.groups) {
            if (group.name == group_name) {
                for (const auto &member : group.members) {
                    members.push_back(member.component);
                }
                break;
            }
        }
        return members;
    }

    /**
     * @brief Get dt for a specific group
     *
     * @param group_name Name of the group
     * @return Time step for that group (global_dt * divisor)
     */
    [[nodiscard]] double GetGroupDt(const std::string &group_name) const {
        double global_dt = 1.0 / sim_rate_hz_;
        auto it = config_.group_frame_divisors.find(group_name);
        if (it != config_.group_frame_divisors.end()) {
            return global_dt * it->second;
        }
        return global_dt;
    }

    /**
     * @brief Get frame divisor for a group
     *
     * @param group_name Name of the group
     * @return Frame divisor (1 = runs every frame)
     */
    [[nodiscard]] int GetFrameDivisor(const std::string &group_name) const {
        auto it = config_.group_frame_divisors.find(group_name);
        if (it != config_.group_frame_divisors.end()) {
            return it->second;
        }
        return 1;
    }

    /// Get simulation rate in Hz
    [[nodiscard]] double GetSimulationRate() const { return sim_rate_hz_; }

    /// Get the sorted groups (by priority)
    [[nodiscard]] const std::vector<SchedulerGroupConfig> &GetGroups() const {
        return config_.groups;
    }

    /// Get the config
    [[nodiscard]] const SchedulerConfig &GetConfig() const { return config_; }

    /// Log execution order to logger or stdout
    void LogExecutionOrder(MissionLogger *logger = nullptr) const {
        if (logger != nullptr) {
            // Use structured MissionLogger output
            logger->LogSchedulerOrder(sim_rate_hz_, config_.groups, config_.group_frame_divisors);
        } else {
            // Fallback to stdout
            std::ostringstream oss;
            oss << "Scheduler execution order (sim rate: " << sim_rate_hz_ << " Hz):\n";

            for (const auto &group : config_.groups) {
                auto it = config_.group_frame_divisors.find(group.name);
                int divisor = (it != config_.group_frame_divisors.end()) ? it->second : 1;
                double group_dt = 1.0 / group.rate_hz;

                oss << "  Group '" << group.name << "' (priority " << group.priority << ", "
                    << group.rate_hz << " Hz, divisor " << divisor << ", dt=" << std::fixed
                    << std::setprecision(6) << group_dt << "s):\n";

                for (const auto &member : group.members) {
                    oss << "    - " << member.component << " (priority " << member.priority
                        << ")\n";
                }
            }
            std::cout << oss.str();
        }
    }

    SchedulerConfig config_;
    double sim_rate_hz_ = 400.0;
};

} // namespace icarus
