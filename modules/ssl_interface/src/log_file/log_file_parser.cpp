#include "log_file/log_file_parser.hpp"

namespace luhsoccer::ssl_interface {

LogFileParser::LogFileParser(const std::string& filename)
    : file_input(filename, std::ios_base::in | std::ios_base::binary),
      i_file_input_stream(google::protobuf::io::IstreamInputStream(&file_input)),
      stream(&i_file_input_stream) {}

std::optional<const void*> LogFileParser::dataAvailable(int min_size) {
    // Calculate the bytes sizes
    const int bytes_available = chunk_size - current_offset;
    const int missing_bytes = min_size - bytes_available;

    // Check if there is a data overlap (the current chunk contains not enough bytes)
    const bool data_overlap = (bytes_available < min_size) && chunk_ptr != nullptr;

    if (data_overlap) {  // Copy the rest of the current chunk into a buffer
        data_buffer.clear();
        data_buffer.insert(data_buffer.end(), chunk_ptr + current_offset, chunk_ptr + chunk_size);
    }

    if (chunk_ptr == nullptr || bytes_available < min_size) {  // Try to read the next chunk of data
        const void* next_data_ptr = nullptr;
        // Read the the chunk from the stream
        if (!stream.Next(&next_data_ptr, &chunk_size)) {
            return std::nullopt;
        }

        chunk_ptr = static_cast<const char*>(next_data_ptr);
        current_offset = 0;

        if (data_overlap) {
            // Insert the rest of the last message into the current buffer
            data_buffer.insert(data_buffer.end(), chunk_ptr, chunk_ptr + missing_bytes);
            current_offset += missing_bytes;
            // Return the combined message
            return std::make_optional(data_buffer.data());
        }
    }
    const auto ptr = std::make_optional(chunk_ptr + current_offset);
    // Increase the offset to match the read bytes
    current_offset += min_size;
    return ptr;
}

std::optional<LogFileHeader> LogFileParser::readHeader() {
    const auto header_ptr = next<LogFileHeader>();
    if (!header_ptr) {
        return std::nullopt;
    }
    return std::make_optional(LogFileHeader{header_ptr.value()->file_type, swapEndian(header_ptr.value()->version)});
}
std::optional<LogFileMessage> LogFileParser::readMessage() {
    const auto header_ptr = next<LogFileMessageHeader>();

    if (!header_ptr) {
        return std::nullopt;
    }

    const auto header =
        LogFileMessageHeader{swapEndian(header_ptr.value()->timestamp), swapEndian(header_ptr.value()->message_type),
                             swapEndian(header_ptr.value()->message_size)};

    if (header.message_type == MESSAGE_SSL_INDEX_2021) {  // If we found an index message we're at end of the log file
        return std::nullopt;
    }
    const auto message_ptr = next(header.message_size);

    if (!message_ptr) {
        return std::nullopt;
    }

    return std::make_optional(LogFileMessage{header, message_ptr.value()});
}

}  // namespace luhsoccer::ssl_interface