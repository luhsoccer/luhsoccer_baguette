#include "log_file/log_file_parser.hpp"
#include "boost/iostreams/filter/gzip.hpp"
#include "boost/iostreams/device/file_descriptor.hpp"

namespace luhsoccer::ssl_interface {
LogFileParser::LogFileParser(const std::string& filename) {
    bool is_gzip = filename.find("gz") != std::string::npos;
    if (is_gzip) {
        stream.push(boost::iostreams::gzip_decompressor(), 2 << 16);
    }
    data_buffer.reserve(2 << 10);
    stream.push(boost::iostreams::file_descriptor_source(filename, std::ios::in | std::ios::binary), 2 << 16);
}

template <typename T>
const T* LogFileParser::next() {
    const auto ptr = dataAvailable(sizeof(T));
    return static_cast<const T*>(ptr);
}

const char* LogFileParser::next(int bytes) {
    const auto ptr = dataAvailable(bytes);
    return static_cast<const char*>(ptr);
}

const void* LogFileParser::dataAvailable(int min_size) {
    auto read = boost::iostreams::read(stream, data_buffer.data(), min_size);

    if (read != min_size) {
        logger.warning("Read {} bytes to needed {}", read, min_size);
        return nullptr;
    }

    return data_buffer.data();
}

std::optional<LogFileHeader> LogFileParser::readHeader() {
    const auto header_ptr = next<LogFileHeader>();
    if (!header_ptr) {
        return std::nullopt;
    }
    return std::make_optional(LogFileHeader{header_ptr->file_type, swapEndian(header_ptr->version)});
}

std::optional<LogFileMessage> LogFileParser::readMessage() {
    const auto header_ptr = next<LogFileMessageHeader>();

    if (!header_ptr) {
        return std::nullopt;
    }

    const auto header = LogFileMessageHeader{swapEndian(header_ptr->timestamp), swapEndian(header_ptr->message_type),
                                             swapEndian(header_ptr->message_size)};

    if (header.message_type == MESSAGE_SSL_INDEX_2021) {
        // If we found an index message we're at end of the log file
        logger.info("Reached end of log file");
        return std::nullopt;
    }

    const auto message_ptr = next(header.message_size);

    if (!message_ptr) {
        return std::nullopt;
    }

    return std::make_optional(LogFileMessage{header, message_ptr});
}
}  // namespace luhsoccer::ssl_interface
