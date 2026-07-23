#pragma once

#include "foundationpose_shm_bridge/control_protocol.hpp"

#include <cstdint>
#include <mutex>
#include <string>

namespace foundationpose_shm_bridge
{
    class ControlShmWriter
    {
    public:
        explicit ControlShmWriter(std::string shm_name, bool unlink_on_exit = true);
        ~ControlShmWriter();

        ControlShmWriter(const ControlShmWriter&) = delete;
        ControlShmWriter& operator=(const ControlShmWriter&) = delete;

        void updateConfig(const ControlConfig& config);
        void requestReinitialize();
        void requestShutdown();

        [[nodiscard]] const std::string& name() const noexcept;

    private:
        void publishLocked();

        std::string shm_name_;
        bool unlink_on_exit_{true};
        int shm_fd_{-1};
        void* mapped_address_{nullptr};
        ControlBlock* block_{nullptr};

        std::mutex mutex_;
        ControlConfig config_{};
        std::uint64_t sequence_counter_{0U};
        std::uint64_t command_counter_{0U};
        std::uint64_t reinitialize_counter_{0U};
        std::uint64_t shutdown_counter_{0U};
    };
} // namespace foundationpose_shm_bridge
