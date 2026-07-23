#pragma once

#include "foundationpose_shm_bridge/shared_memory_protocol.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace foundationpose_shm_bridge
{
    // SharedRgbdWriter 只负责一件事：
    // 将一帧 RGB、Depth、K 和时间戳安全地写进 POSIX 共享内存。
    class SharedRgbdWriter
    {
    public:
        SharedRgbdWriter(
            std::string shm_name,
            std::uint32_t width,
            std::uint32_t height,
            bool unlink_on_exit);

        ~SharedRgbdWriter();

        // 共享内存对象不能复制，否则多个对象会重复 munmap/close。
        SharedRgbdWriter(const SharedRgbdWriter&) = delete;
        SharedRgbdWriter& operator=(const SharedRgbdWriter&) = delete;
        SharedRgbdWriter(SharedRgbdWriter&&) = delete;
        SharedRgbdWriter& operator=(SharedRgbdWriter&&) = delete;

        void write(
            const cv::Mat& rgb,
            const cv::Mat& depth_m,
            const std::array<double, 9>& camera_k,
            std::uint64_t rgb_timestamp_ns,
            std::uint64_t depth_timestamp_ns);

        [[nodiscard]] std::uint32_t width() const noexcept;
        [[nodiscard]] std::uint32_t height() const noexcept;
        [[nodiscard]] std::size_t mappedSize() const noexcept;
        [[nodiscard]] const std::string& name() const noexcept;

    private:
        [[nodiscard]] std::size_t slotBaseOffset(std::uint32_t slot_index) const;
        [[nodiscard]] SlotHeader* slotHeader(std::uint32_t slot_index) const;
        [[nodiscard]] std::uint8_t* rgbAddress(std::uint32_t slot_index) const;
        [[nodiscard]] float* depthAddress(std::uint32_t slot_index) const;

        static void copyMatRows(
            const cv::Mat& source,
            void* destination,
            std::size_t row_bytes);

        std::string shm_name_;
        bool unlink_on_exit_{false};

        int shm_fd_{-1};
        void* mapped_address_{nullptr};
        GlobalHeader* global_header_{nullptr};

        std::size_t rgb_bytes_{0U};
        std::size_t depth_bytes_{0U};
        std::size_t rgb_offset_in_slot_{0U};
        std::size_t depth_offset_in_slot_{0U};
        std::size_t slot_stride_{0U};
        std::size_t mapped_size_{0U};

        std::uint64_t frame_counter_{0U};
    };
} // namespace foundationpose_shm_bridge
