#include "ssl_interface/log_file.hpp"
#include "log_file/log_file_parser.hpp"
#include "ssl_vision_wrapper.pb.h"
#include "ssl_gc_referee_message.pb.h"
#include <fstream>
#include "ssl_types_converter/ssl_types_converter.hpp"

namespace luhsoccer::ssl_interface {

LogFile::LogFile(std::string file) : file(std::move(file)) {}

void LogFile::load() {
    packets.clear();
    LogFileParser parser{file};

    parser.readHeader();

    while (true) {
        const auto message = parser.readMessage();
        if (!message) {
            LOG_INFO(logger, "Parsed logfile {} successfully", file);
            break;
        } else {
            switch (message->header.message_type) {
                case LogFileParser::MESSAGE_SSL_REFBOX_2013:
                    addPacket<Referee, SSLGameControllerData>(message->message, message->header.message_size,
                                                              converter::parseRefereeData);
                    break;
                case LogFileParser::MESSAGE_SSL_VISION_2014:
                    addPacket<proto::ssl_vision::SSL_WrapperPacket, SSLWrapperData>(
                        message->message, message->header.message_size, converter::parseWrapperData);
                    break;
                default:
                    // Ignore all other messages by default
                    break;
            }
        }
    }
}

void LogFile::seekBy(int offset) { current_pos += offset; }

void LogFile::replayAtMaxSpeed(SSLInterface& interface) {
    current_pos = 0;
    while (current_pos < packets.size()) {
        const auto& packet = packets[current_pos];
        if (std::holds_alternative<SSLWrapperData>(packet)) {
            interface.processWrapperData<VisionDataSource::GAME_LOG>(std::get<SSLWrapperData>(packet));
        } else if (std::holds_alternative<SSLGameControllerData>(packet)) {
            interface.processGameControllerData<GameControllerDataSource::GAME_LOG>(
                std::get<SSLGameControllerData>(packet));
        }
        current_pos++;
    }
}

}  // namespace luhsoccer::ssl_interface
