#pragma once

#include "foundationpose_shm_bridge/result_protocol.hpp"

#include <cstdint>
#include <string>

namespace foundationpose_shm_bridge
{
    class ResultShmReader
    {
    public:
        explicit ResultShmReader(std::string shm_name);
        ~ResultShmReader();

        ResultShmReader(const ResultShmReader&) = delete;
        ResultShmReader& operator=(const ResultShmReader&) = delete;

        bool tryOpen();
        void close();
        [[nodiscard]] bool isOpen() const noexcept;
        [[nodiscard]] bool mappingIsCurrent() const;
        bool readLatest(std::uint64_t last_sequence, ResultFrame& result, int max_retries = 5);

    private:
        [[nodiscard]] std::string path() const;
        [[nodiscard]] std::size_t slotBaseOffset(std::uint32_t slot_index) const;

        std::string shm_name_;
        int shm_fd_{-1};
        void* mapped_address_{nullptr};
        std::size_t mapped_size_{0U};
        std::uint64_t inode_{0U};

        const ResultGlobalHeader* global_header_{nullptr};
        std::uint32_t width_{0U};
        std::uint32_t height_{0U};
        std::uint32_t slot_count_{0U};
        std::size_t debug_bytes_{0U};
        std::size_t mask_bytes_{0U};
        std::size_t debug_offset_in_slot_{0U};
        std::size_t mask_offset_in_slot_{0U};
        std::size_t slot_stride_{0U};
    };
} // namespace foundationpose_shm_bridge
