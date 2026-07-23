#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace foundationpose_shm_bridge
{
    constexpr std::uint64_t kResultMagic = 0x4650524553554C31ULL; // "FPRESUL1"
    constexpr std::uint32_t kResultProtocolVersion = 1U;
    constexpr std::uint32_t kResultSlotCount = 2U;
    constexpr std::size_t kResultAlignment = 64U;

    struct alignas(64) ResultGlobalHeader
    {
        std::uint64_t magic;
        std::uint32_t version;
        std::uint32_t header_size;
        std::uint32_t width;
        std::uint32_t height;
        std::uint32_t slot_count;
        std::uint32_t active_slot;
        std::uint64_t debug_bytes;
        std::uint64_t mask_bytes;
        std::uint64_t debug_offset_in_slot;
        std::uint64_t mask_offset_in_slot;
        std::uint64_t slot_stride;
        std::uint64_t latest_sequence;
        std::uint8_t reserved[48];
    };

    static_assert(sizeof(ResultGlobalHeader) == 128U, "ResultGlobalHeader must be 128 bytes");

    struct alignas(64) ResultSlotHeader
    {
        std::uint64_t sequence;
        std::uint64_t frame_index;
        std::uint64_t source_timestamp_ns;
        std::uint64_t result_timestamp_ns;
        std::uint32_t state;
        std::uint32_t pose_valid;
        std::uint32_t debug_valid;
        std::uint32_t mask_valid;
        double pose[16];
        float inference_ms;
        float fps;
        float yolo_confidence;
        std::uint32_t reserved0;
        char status[64];
    };

    static_assert(sizeof(ResultSlotHeader) == 256U, "ResultSlotHeader must be 256 bytes");

    struct ResultFrame
    {
        std::uint64_t sequence{0U};
        std::uint64_t frame_index{0U};
        std::uint64_t source_timestamp_ns{0U};
        std::uint64_t result_timestamp_ns{0U};
        std::uint32_t state{0U};
        bool pose_valid{false};
        bool debug_valid{false};
        bool mask_valid{false};
        std::array<double, 16> pose{};
        float inference_ms{0.0F};
        float fps{0.0F};
        float yolo_confidence{0.0F};
        std::string status;
        cv::Mat debug_bgr;
        cv::Mat mask_mono8;
    };

    constexpr std::size_t resultAlignUp(const std::size_t value, const std::size_t alignment)
    {
        return (value + alignment - 1U) / alignment * alignment;
    }
} // namespace foundationpose_shm_bridge
