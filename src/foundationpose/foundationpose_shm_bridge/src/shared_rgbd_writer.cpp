#include "foundationpose_shm_bridge/shared_rgbd_writer.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace foundationpose_shm_bridge
{
    namespace
    {
        std::runtime_error makeSystemError(const std::string& operation)
        {
            return std::runtime_error(operation + " failed: " + std::strerror(errno));
        }

        std::uint32_t checkedUint32(const std::size_t value, const char* field_name)
        {
            if (value > std::numeric_limits<std::uint32_t>::max())
            {
                throw std::overflow_error(std::string(field_name) + " exceeds uint32 range");
            }
            return static_cast<std::uint32_t>(value);
        }
    } // namespace

    SharedRgbdWriter::SharedRgbdWriter(
        std::string shm_name,
        const std::uint32_t width,
        const std::uint32_t height,
        const bool unlink_on_exit)
        : shm_name_(std::move(shm_name)), unlink_on_exit_(unlink_on_exit)
    {
        // POSIX 共享内存名称必须以 / 开头，但后面不能再包含路径层级。
        if (shm_name_.empty() || shm_name_.front() != '/' ||
            shm_name_.find('/', 1U) != std::string::npos)
        {
            throw std::invalid_argument(
                "POSIX shared memory name must look like '/foundationpose_rgbd'");
        }

        if (width == 0U || height == 0U)
        {
            throw std::invalid_argument("Image width and height must be positive");
        }

        const std::size_t pixel_count =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

        rgb_bytes_ = pixel_count * 3U * sizeof(std::uint8_t);
        depth_bytes_ = pixel_count * sizeof(float);

        // 每一部分都按 64 字节对齐，保证下一个槽位头部仍然对齐，
        // 也使原子读写和 CPU cache line 行为更可控。
        rgb_offset_in_slot_ = alignUp(sizeof(SlotHeader), kAlignment);
        depth_offset_in_slot_ = alignUp(rgb_offset_in_slot_ + rgb_bytes_, kAlignment);
        slot_stride_ = alignUp(depth_offset_in_slot_ + depth_bytes_, kAlignment);
        mapped_size_ = sizeof(GlobalHeader) + kSlotCount * slot_stride_;

        // 删除上一次异常退出可能留下的旧共享内存名称。
        // 已经 mmap 旧对象的进程仍能访问旧内存，但新进程会打开本次的新对象。
        if (shm_unlink(shm_name_.c_str()) == -1 && errno != ENOENT)
        {
            throw makeSystemError("shm_unlink stale object");
        }

        shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
        if (shm_fd_ == -1)
        {
            throw makeSystemError("shm_open");
        }

        if (ftruncate(shm_fd_, static_cast<off_t>(mapped_size_)) == -1)
        {
            const auto error = makeSystemError("ftruncate");
            close(shm_fd_);
            shm_fd_ = -1;
            shm_unlink(shm_name_.c_str());
            throw error;
        }

        mapped_address_ = mmap(
            nullptr,
            mapped_size_,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shm_fd_,
            0);

        if (mapped_address_ == MAP_FAILED)
        {
            mapped_address_ = nullptr;
            const auto error = makeSystemError("mmap");
            close(shm_fd_);
            shm_fd_ = -1;
            shm_unlink(shm_name_.c_str());
            throw error;
        }

        // 新创建的共享内存理论上已经清零；这里再次显式清零，使初始化语义清楚。
        std::memset(mapped_address_, 0, mapped_size_);

        global_header_ = static_cast<GlobalHeader*>(mapped_address_);
        global_header_->magic = kMagic;
        global_header_->version = kProtocolVersion;
        global_header_->header_size = checkedUint32(sizeof(GlobalHeader), "header_size");
        global_header_->width = width;
        global_header_->height = height;
        global_header_->slot_count = kSlotCount;
        global_header_->rgb_bytes = rgb_bytes_;
        global_header_->depth_bytes = depth_bytes_;
        global_header_->rgb_offset_in_slot = rgb_offset_in_slot_;
        global_header_->depth_offset_in_slot = depth_offset_in_slot_;
        global_header_->slot_stride = slot_stride_;

        // 使用 GCC/Clang 原子内建函数直接操作 mmap 中的整数。
        // RELEASE 保证之前写入的协议字段在 active_slot/latest_sequence 更新前可见。
        __atomic_store_n(&global_header_->active_slot, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&global_header_->latest_sequence, 0ULL, __ATOMIC_RELEASE);
    }

    SharedRgbdWriter::~SharedRgbdWriter()
    {
        if (mapped_address_ != nullptr)
        {
            munmap(mapped_address_, mapped_size_);
            mapped_address_ = nullptr;
            global_header_ = nullptr;
        }

        if (shm_fd_ != -1)
        {
            close(shm_fd_);
            shm_fd_ = -1;
        }

        // 默认不删除，方便接收端在发送节点短暂退出后检查最后一帧。
        // 开启该参数时，进程退出会删除共享内存名称。
        if (unlink_on_exit_)
        {
            shm_unlink(shm_name_.c_str());
        }
    }

    std::uint32_t SharedRgbdWriter::width() const noexcept
    {
        return global_header_->width;
    }

    std::uint32_t SharedRgbdWriter::height() const noexcept
    {
        return global_header_->height;
    }

    std::size_t SharedRgbdWriter::mappedSize() const noexcept
    {
        return mapped_size_;
    }

    const std::string& SharedRgbdWriter::name() const noexcept
    {
        return shm_name_;
    }

    std::size_t SharedRgbdWriter::slotBaseOffset(const std::uint32_t slot_index) const
    {
        if (slot_index >= kSlotCount)
        {
            throw std::out_of_range("Invalid shared memory slot index");
        }

        return sizeof(GlobalHeader) + static_cast<std::size_t>(slot_index) * slot_stride_;
    }

    SlotHeader* SharedRgbdWriter::slotHeader(const std::uint32_t slot_index) const
    {
        auto* base = static_cast<std::uint8_t*>(mapped_address_);
        return reinterpret_cast<SlotHeader*>(base + slotBaseOffset(slot_index));
    }

    std::uint8_t* SharedRgbdWriter::rgbAddress(const std::uint32_t slot_index) const
    {
        auto* base = static_cast<std::uint8_t*>(mapped_address_);
        return base + slotBaseOffset(slot_index) + rgb_offset_in_slot_;
    }

    float* SharedRgbdWriter::depthAddress(const std::uint32_t slot_index) const
    {
        auto* base = static_cast<std::uint8_t*>(mapped_address_);
        return reinterpret_cast<float*>(
            base + slotBaseOffset(slot_index) + depth_offset_in_slot_);
    }

    void SharedRgbdWriter::copyMatRows(
        const cv::Mat& source,
        void* destination,
        const std::size_t row_bytes)
    {
        auto* destination_bytes = static_cast<std::uint8_t*>(destination);

        // 连续 Mat 可以一次 memcpy；带 padding 或 ROI 的 Mat 按行复制。
        if (source.isContinuous() && source.step == row_bytes)
        {
            std::memcpy(
                destination_bytes,
                source.data,
                row_bytes * static_cast<std::size_t>(source.rows));
            return;
        }

        for (int row = 0; row < source.rows; ++row)
        {
            std::memcpy(
                destination_bytes + static_cast<std::size_t>(row) * row_bytes,
                source.ptr(row),
                row_bytes);
        }
    }

    void SharedRgbdWriter::write(
        const cv::Mat& rgb,
        const cv::Mat& depth_m,
        const std::array<double, 9>& camera_k,
        const std::uint64_t rgb_timestamp_ns,
        const std::uint64_t depth_timestamp_ns)
    {
        const int expected_width = static_cast<int>(global_header_->width);
        const int expected_height = static_cast<int>(global_header_->height);

        if (rgb.rows != expected_height || rgb.cols != expected_width || rgb.type() != CV_8UC3)
        {
            throw std::invalid_argument("RGB must be HxW CV_8UC3 in RGB channel order");
        }

        if (depth_m.rows != expected_height || depth_m.cols != expected_width ||
            depth_m.type() != CV_32FC1)
        {
            throw std::invalid_argument("Depth must be HxW CV_32FC1 in meters");
        }

        const std::uint32_t active_slot =
            __atomic_load_n(&global_header_->active_slot, __ATOMIC_ACQUIRE);
        const std::uint32_t write_slot = (active_slot + 1U) % kSlotCount;

        SlotHeader* const slot_header = slotHeader(write_slot);

        ++frame_counter_;
        const std::uint64_t writing_sequence = frame_counter_ * 2ULL - 1ULL;
        const std::uint64_t completed_sequence = frame_counter_ * 2ULL;

        // 第一步：写奇数序号。读取端看到奇数就知道该槽位还不能读。
        __atomic_store_n(&slot_header->sequence, writing_sequence, __ATOMIC_RELEASE);

        slot_header->frame_index = frame_counter_;
        slot_header->rgb_timestamp_ns = rgb_timestamp_ns;
        slot_header->depth_timestamp_ns = depth_timestamp_ns;
        std::memcpy(slot_header->camera_k, camera_k.data(), sizeof(slot_header->camera_k));

        copyMatRows(
            rgb,
            rgbAddress(write_slot),
            static_cast<std::size_t>(expected_width) * 3U * sizeof(std::uint8_t));

        copyMatRows(
            depth_m,
            depthAddress(write_slot),
            static_cast<std::size_t>(expected_width) * sizeof(float));

        // 第二步：所有数据完成后，把槽位序号改为偶数。
        __atomic_store_n(&slot_header->sequence, completed_sequence, __ATOMIC_RELEASE);

        // 第三步：最后才公布 active_slot 和 latest_sequence。
        // 因此读取端只要看到新的 latest_sequence，就能找到完整帧。
        __atomic_store_n(&global_header_->active_slot, write_slot, __ATOMIC_RELEASE);
        __atomic_store_n(&global_header_->latest_sequence, completed_sequence, __ATOMIC_RELEASE);
    }
} // namespace foundationpose_shm_bridge
