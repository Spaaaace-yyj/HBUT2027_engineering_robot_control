#include "foundationpose_shm_bridge/control_shm_writer.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
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

        void validateName(const std::string& name)
        {
            if (name.empty() || name.front() != '/' || name.find('/', 1U) != std::string::npos)
            {
                throw std::invalid_argument("POSIX shared memory name must look like '/foundationpose_control'");
            }
        }
    } // namespace

    ControlShmWriter::ControlShmWriter(std::string shm_name, const bool unlink_on_exit)
        : shm_name_(std::move(shm_name)), unlink_on_exit_(unlink_on_exit)
    {
        validateName(shm_name_);

        if (shm_unlink(shm_name_.c_str()) == -1 && errno != ENOENT)
        {
            throw makeSystemError("shm_unlink stale control object");
        }

        shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
        if (shm_fd_ == -1)
        {
            throw makeSystemError("shm_open control");
        }

        if (ftruncate(shm_fd_, static_cast<off_t>(sizeof(ControlBlock))) == -1)
        {
            const auto error = makeSystemError("ftruncate control");
            close(shm_fd_);
            shm_fd_ = -1;
            shm_unlink(shm_name_.c_str());
            throw error;
        }

        mapped_address_ = mmap(
            nullptr,
            sizeof(ControlBlock),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shm_fd_,
            0);

        if (mapped_address_ == MAP_FAILED)
        {
            mapped_address_ = nullptr;
            const auto error = makeSystemError("mmap control");
            close(shm_fd_);
            shm_fd_ = -1;
            shm_unlink(shm_name_.c_str());
            throw error;
        }

        std::memset(mapped_address_, 0, sizeof(ControlBlock));
        block_ = static_cast<ControlBlock*>(mapped_address_);
        block_->magic = kControlMagic;
        block_->version = kControlProtocolVersion;
        block_->block_size = static_cast<std::uint32_t>(sizeof(ControlBlock));

        publishLocked();
    }

    ControlShmWriter::~ControlShmWriter()
    {
        if (mapped_address_ != nullptr)
        {
            munmap(mapped_address_, sizeof(ControlBlock));
            mapped_address_ = nullptr;
            block_ = nullptr;
        }

        if (shm_fd_ != -1)
        {
            close(shm_fd_);
            shm_fd_ = -1;
        }

        if (unlink_on_exit_)
        {
            shm_unlink(shm_name_.c_str());
        }
    }

    void ControlShmWriter::updateConfig(const ControlConfig& config)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        ++command_counter_;
        publishLocked();
    }

    void ControlShmWriter::requestReinitialize()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++reinitialize_counter_;
        ++command_counter_;
        publishLocked();
    }

    void ControlShmWriter::requestShutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++shutdown_counter_;
        ++command_counter_;
        publishLocked();
    }

    const std::string& ControlShmWriter::name() const noexcept
    {
        return shm_name_;
    }

    void ControlShmWriter::publishLocked()
    {
        sequence_counter_ += 2U;
        const std::uint64_t writing_sequence = sequence_counter_ - 1U;
        const std::uint64_t completed_sequence = sequence_counter_;

        __atomic_store_n(&block_->sequence, writing_sequence, __ATOMIC_RELEASE);

        block_->command_counter = command_counter_;
        block_->reinitialize_counter = reinitialize_counter_;
        block_->shutdown_counter = shutdown_counter_;

        block_->enabled = config_.enabled ? 1U : 0U;
        block_->yolo_imgsz = config_.yolo_imgsz;
        block_->est_refine_iter = config_.est_refine_iter;
        block_->track_refine_iter = config_.track_refine_iter;
        block_->min_mask_pixels = config_.min_mask_pixels;
        block_->mask_close_kernel = config_.mask_close_kernel;
        block_->publish_debug_image = config_.publish_debug_image ? 1U : 0U;
        block_->publish_mask = config_.publish_mask ? 1U : 0U;

        block_->yolo_conf = config_.yolo_conf;
        block_->mask_threshold = config_.mask_threshold;
        block_->min_depth_m = config_.min_depth_m;
        block_->max_depth_m = config_.max_depth_m;
        block_->min_valid_depth_ratio = config_.min_valid_depth_ratio;

        __atomic_store_n(&block_->sequence, completed_sequence, __ATOMIC_RELEASE);
    }
} // namespace foundationpose_shm_bridge
