#pragma once

#include <array>
#include <optional>
#include "boost/iostreams/filtering_streambuf.hpp"
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
    const T* next();

    const char* next(int bytes);

    const void* dataAvailable(int min_size = 1);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> stream;
    std::vector<char> data_buffer;
    logger::Logger logger{"log_file_parser"};
};
}  // namespace luhsoccer::ssl_interface
