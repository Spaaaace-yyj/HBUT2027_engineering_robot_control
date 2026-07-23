#include "foundationpose_shm_bridge/result_shm_reader.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace foundationpose_shm_bridge
{
    namespace
    {
        void validateName(const std::string& name)
        {
            if (name.empty() || name.front() != '/' || name.find('/', 1U) != std::string::npos)
            {
                throw std::invalid_argument("POSIX shared memory name must look like '/foundationpose_result'");
            }
        }
    } // namespace

    ResultShmReader::ResultShmReader(std::string shm_name)
        : shm_name_(std::move(shm_name))
    {
        validateName(shm_name_);
    }

    ResultShmReader::~ResultShmReader()
    {
        close();
    }

    std::string ResultShmReader::path() const
    {
        return "/dev/shm/" + shm_name_.substr(1U);
    }

    bool ResultShmReader::tryOpen()
    {
        if (isOpen())
        {
            return true;
        }

        shm_fd_ = open(path().c_str(), O_RDONLY);
        if (shm_fd_ == -1)
        {
            return false;
        }

        struct stat info{};
        if (fstat(shm_fd_, &info) == -1 || info.st_size < static_cast<off_t>(sizeof(ResultGlobalHeader)))
        {
            close();
            return false;
        }

        mapped_size_ = static_cast<std::size_t>(info.st_size);
        inode_ = static_cast<std::uint64_t>(info.st_ino);
        mapped_address_ = mmap(nullptr, mapped_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
        if (mapped_address_ == MAP_FAILED)
        {
            mapped_address_ = nullptr;
            close();
            return false;
        }

        global_header_ = static_cast<const ResultGlobalHeader*>(mapped_address_);
        if (global_header_->magic != kResultMagic ||
            global_header_->version != kResultProtocolVersion ||
            global_header_->header_size != sizeof(ResultGlobalHeader) ||
            global_header_->slot_count != kResultSlotCount)
        {
            close();
            return false;
        }

        width_ = global_header_->width;
        height_ = global_header_->height;
        slot_count_ = global_header_->slot_count;
        debug_bytes_ = static_cast<std::size_t>(global_header_->debug_bytes);
        mask_bytes_ = static_cast<std::size_t>(global_header_->mask_bytes);
        debug_offset_in_slot_ = static_cast<std::size_t>(global_header_->debug_offset_in_slot);
        mask_offset_in_slot_ = static_cast<std::size_t>(global_header_->mask_offset_in_slot);
        slot_stride_ = static_cast<std::size_t>(global_header_->slot_stride);

        const std::size_t required_size = sizeof(ResultGlobalHeader) + slot_count_ * slot_stride_;
        if (width_ == 0U || height_ == 0U || required_size > mapped_size_)
        {
            close();
            return false;
        }

        return true;
    }

    void ResultShmReader::close()
    {
        if (mapped_address_ != nullptr)
        {
            munmap(mapped_address_, mapped_size_);
        }
        mapped_address_ = nullptr;
        global_header_ = nullptr;
        mapped_size_ = 0U;
        inode_ = 0U;

        if (shm_fd_ != -1)
        {
            ::close(shm_fd_);
            shm_fd_ = -1;
        }
    }

    bool ResultShmReader::isOpen() const noexcept
    {
        return mapped_address_ != nullptr;
    }

    bool ResultShmReader::mappingIsCurrent() const
    {
        if (!isOpen())
        {
            return false;
        }

        struct stat info{};
        if (stat(path().c_str(), &info) == -1)
        {
            return false;
        }
        return static_cast<std::uint64_t>(info.st_ino) == inode_ &&
            static_cast<std::size_t>(info.st_size) == mapped_size_;
    }

    std::size_t ResultShmReader::slotBaseOffset(const std::uint32_t slot_index) const
    {
        return sizeof(ResultGlobalHeader) + static_cast<std::size_t>(slot_index) * slot_stride_;
    }

    bool ResultShmReader::readLatest(
        const std::uint64_t last_sequence,
        ResultFrame& result,
        const int max_retries)
    {
        if (!isOpen())
        {
            return false;
        }

        const auto* base = static_cast<const std::uint8_t*>(mapped_address_);

        for (int attempt = 0; attempt < max_retries; ++attempt)
        {
            const std::uint32_t active_slot =
                __atomic_load_n(&global_header_->active_slot, __ATOMIC_ACQUIRE);
            const std::uint64_t latest_sequence =
                __atomic_load_n(&global_header_->latest_sequence, __ATOMIC_ACQUIRE);

            if (latest_sequence == 0U || latest_sequence == last_sequence || active_slot >= slot_count_)
            {
                return false;
            }

            const std::size_t slot_base = slotBaseOffset(active_slot);
            const auto* slot = reinterpret_cast<const ResultSlotHeader*>(base + slot_base);
            const std::uint64_t sequence_before =
                __atomic_load_n(&slot->sequence, __ATOMIC_ACQUIRE);

            if (sequence_before == 0U || (sequence_before & 1U) != 0U || sequence_before != latest_sequence)
            {
                continue;
            }

            ResultFrame candidate;
            candidate.sequence = sequence_before;
            candidate.frame_index = slot->frame_index;
            candidate.source_timestamp_ns = slot->source_timestamp_ns;
            candidate.result_timestamp_ns = slot->result_timestamp_ns;
            candidate.state = slot->state;
            candidate.pose_valid = slot->pose_valid != 0U;
            candidate.debug_valid = slot->debug_valid != 0U;
            candidate.mask_valid = slot->mask_valid != 0U;
            std::copy(std::begin(slot->pose), std::end(slot->pose), candidate.pose.begin());
            candidate.inference_ms = slot->inference_ms;
            candidate.fps = slot->fps;
            candidate.yolo_confidence = slot->yolo_confidence;
            candidate.status.assign(slot->status, strnlen(slot->status, sizeof(slot->status)));

            if (candidate.debug_valid)
            {
                candidate.debug_bgr.create(static_cast<int>(height_), static_cast<int>(width_), CV_8UC3);
                std::memcpy(candidate.debug_bgr.data, base + slot_base + debug_offset_in_slot_, debug_bytes_);
            }

            if (candidate.mask_valid)
            {
                candidate.mask_mono8.create(static_cast<int>(height_), static_cast<int>(width_), CV_8UC1);
                std::memcpy(candidate.mask_mono8.data, base + slot_base + mask_offset_in_slot_, mask_bytes_);
            }

            const std::uint64_t sequence_after =
                __atomic_load_n(&slot->sequence, __ATOMIC_ACQUIRE);
            const std::uint32_t active_after =
                __atomic_load_n(&global_header_->active_slot, __ATOMIC_ACQUIRE);
            const std::uint64_t latest_after =
                __atomic_load_n(&global_header_->latest_sequence, __ATOMIC_ACQUIRE);

            if (sequence_before == sequence_after &&
                (sequence_after & 1U) == 0U &&
                active_after == active_slot &&
                latest_after == sequence_after)
            {
                result = std::move(candidate);
                return true;
            }
        }

        return false;
    }
} // namespace foundationpose_shm_bridge
