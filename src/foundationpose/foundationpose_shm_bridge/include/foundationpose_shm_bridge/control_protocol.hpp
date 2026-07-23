#pragma once

#include <cstddef>
#include <cstdint>

namespace foundationpose_shm_bridge
{
    constexpr std::uint64_t kControlMagic = 0x46504354524C3031ULL; // "FPCTRL01"
    constexpr std::uint32_t kControlProtocolVersion = 1U;
    constexpr std::size_t kControlBlockSize = 256U;

    // C++ ROS Bridge 写、Python Worker 读。
    // sequence 使用奇偶 seqlock：奇数=正在写，偶数=完整快照。
    struct alignas(64) ControlBlock
    {
        std::uint64_t magic; // 0
        std::uint32_t version; // 8
        std::uint32_t block_size; // 12
        std::uint64_t sequence; // 16
        std::uint64_t command_counter; // 24
        std::uint64_t reinitialize_counter; // 32
        std::uint64_t shutdown_counter; // 40

        std::uint32_t enabled; // 48
        std::uint32_t yolo_imgsz; // 52
        std::uint32_t est_refine_iter; // 56
        std::uint32_t track_refine_iter; // 60
        std::uint32_t min_mask_pixels; // 64
        std::uint32_t mask_close_kernel; // 68
        std::uint32_t publish_debug_image; // 72
        std::uint32_t publish_mask; // 76

        double yolo_conf; // 80
        double mask_threshold; // 88
        double min_depth_m; // 96
        double max_depth_m; // 104
        double min_valid_depth_ratio; // 112

        std::uint8_t reserved[136]; // 120..255
    };

    static_assert(sizeof(ControlBlock) == kControlBlockSize, "ControlBlock must be 256 bytes");

    struct ControlConfig
    {
        bool enabled{true};
        std::uint32_t yolo_imgsz{640U};
        std::uint32_t est_refine_iter{5U};
        std::uint32_t track_refine_iter{2U};
        std::uint32_t min_mask_pixels{500U};
        std::uint32_t mask_close_kernel{0U};
        bool publish_debug_image{true};
        bool publish_mask{true};
        double yolo_conf{0.5};
        double mask_threshold{0.5};
        double min_depth_m{0.05};
        double max_depth_m{5.0};
        double min_valid_depth_ratio{0.05};
    };
} // namespace foundationpose_shm_bridge
