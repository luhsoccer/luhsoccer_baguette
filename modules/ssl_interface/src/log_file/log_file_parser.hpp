#pragma once

#include <array>
#include <fstream>
#include <optional>
#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "logger/logger.hpp"

namespace luhsoccer::ssl_interface {

// dervied from https://stackoverflow.com/a/4956493
template <typename T>
T swapEndian(T u) {
    union {
        T u;
        std::array<unsigned char, sizeof(T)> u8;
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++) dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

struct LogFileHeader {
    constexpr static size_t HEADER_SIZE = 12;
    std::array<unsigned char, HEADER_SIZE> file_type;
    int32_t version;
};

struct LogFileMessageHeader {
    int64_t timestamp;
    int32_t message_type;
    int32_t message_size;
};

struct LogFileMessage {
    LogFileMessageHeader header;
    const char* message;
};

class LogFileParser {
   public:
    constexpr inline static int MESSAGE_BLANK = 0;
    constexpr inline static int MESSAGE_UNKNOWN = 1;
    constexpr inline static int MESSAGE_SSL_VISION_2010 = 2;
    constexpr inline static int MESSAGE_SSL_REFBOX_2013 = 3;
    constexpr inline static int MESSAGE_SSL_VISION_2014 = 4;
    constexpr inline static int MESSAGE_SSL_VISION_TRACKER_2020 = 5;
    constexpr inline static int MESSAGE_SSL_INDEX_2021 = 6;

    LogFileParser(const std::string& file);

    std::optional<LogFileHeader> readHeader();
    std::optional<LogFileMessage> readMessage();

   private:
    template <typename T>
    std::optional<const T*> next() {
        const auto ptr = dataAvailable(sizeof(T));
        if (!ptr) {
            return std::nullopt;
        }
        return std::make_optional(static_cast<const T*>(ptr.value()));
    }

    std::optional<const char*> next(int bytes) {
        const auto ptr = dataAvailable(bytes);
        if (!ptr) {
            return std::nullopt;
        }
        return std::make_optional(static_cast<const char*>(ptr.value()));
    }
    std::optional<const void*> dataAvailable(int min_size = 1);

    std::ifstream file_input;
    google::protobuf::io::IstreamInputStream i_file_input_stream;
    google::protobuf::io::GzipInputStream stream;
    int chunk_size{0};
    int current_offset{0};
    std::vector<char> data_buffer;
    const char* chunk_ptr = nullptr;
    logger::Logger logger{"log_file_parser"};
};

}  // namespace luhsoccer::ssl_interface