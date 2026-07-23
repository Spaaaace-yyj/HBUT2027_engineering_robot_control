#pragma once

#include <cstddef>
#include <cstdint>

namespace foundationpose_shm_bridge
{
    // -----------------------------------------------------------------------------
    // C++ 与 Python 必须共同遵守的共享内存二进制协议。
    //
    // 设计原则：
    // 1. 只使用固定宽度整数和 double，避免不同编译器下类型长度不一致。
    // 2. GlobalHeader 和 SlotHeader 都固定为 128 字节，便于 Python 按偏移解析。
    // 3. RGB 固定为 uint8 RGB 排列，Depth 固定为 float32 米。
    // 4. 使用两个槽位（双缓冲），写入端永远写“非活动槽位”。
    // -----------------------------------------------------------------------------

    constexpr std::uint64_t kMagic = 0x4650524742443031ULL;
    constexpr std::uint32_t kProtocolVersion = 1U;
    constexpr std::uint32_t kSlotCount = 2U;
    constexpr std::size_t kAlignment = 64U;

    // 全局头部描述共享内存的整体布局。
    // active_slot 和 latest_sequence 会被写入端原子更新。
    struct alignas(64) GlobalHeader
    {
        std::uint64_t magic; // 协议魔数，用于确认映射的是正确文件
        std::uint32_t version; // 协议版本
        std::uint32_t header_size; // sizeof(GlobalHeader)

        std::uint32_t width; // 图像宽度
        std::uint32_t height; // 图像高度
        std::uint32_t slot_count; // 当前固定为 2
        std::uint32_t active_slot; // 最近一次完整写入的槽位编号

        std::uint64_t rgb_bytes; // 单帧 RGB 字节数
        std::uint64_t depth_bytes; // 单帧 Depth 字节数
        std::uint64_t rgb_offset_in_slot; // RGB 相对槽位起点的偏移
        std::uint64_t depth_offset_in_slot; // Depth 相对槽位起点的偏移
        std::uint64_t slot_stride; // 相邻两个槽位起点的间距
        std::uint64_t latest_sequence; // 最近完整帧的偶数序号

        std::uint8_t reserved[48]; // 预留，保证结构体固定为 128 字节
    };

    static_assert(sizeof(GlobalHeader) == 128U, "GlobalHeader must be 128 bytes");

    // 每个槽位自己的元数据。
    // sequence 为奇数：该槽位正在写；sequence 为偶数：该槽位写入完成。
    struct alignas(64) SlotHeader
    {
        std::uint64_t sequence;
        std::uint64_t frame_index;
        std::uint64_t rgb_timestamp_ns;
        std::uint64_t depth_timestamp_ns;

        // 3x3 相机内参矩阵，按行排列：
        // [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        double camera_k[9];

        std::uint8_t reserved[24];
    };

    static_assert(sizeof(SlotHeader) == 128U, "SlotHeader must be 128 bytes");

    constexpr std::size_t alignUp(const std::size_t value, const std::size_t alignment)
    {
        return (value + alignment - 1U) / alignment * alignment;
    }
} // namespace foundationpose_shm_bridge
