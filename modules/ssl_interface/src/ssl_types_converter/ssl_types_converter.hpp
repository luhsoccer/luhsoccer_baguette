#pragma once
#include "ssl_interface/ssl_types.hpp"
#include "ssl_gc_common.pb.h"
#include "ssl_gc_referee_message.pb.h"
#include "ssl_vision_wrapper.pb.h"

namespace luhsoccer::ssl_interface::converter {

using namespace proto;

static constexpr double SCALE_FACTOR =
    0.001;  // ssl_vision reports sizes and scales in mm, our baguette is measured in meters

/**
 * @brief A simple functions that performs the same action for two distinct input/output pairs
 *
 * @tparam Input The type of the inputs
 * @tparam Output The type of the outputs
 * @param input_blue the input data for the first color
 * @param input_yellow the input data for the second color
 * @param out_blue the output struct for the first color
 * @param out_yellow the output struct for the second color
 * @param callback the function which executes the processing
 */
template <typename Input, typename Output>
void doForBothColors(const Input& input_blue, const Input& input_yellow, Output& out_blue, Output& out_yellow,
                     std::function<void(const Input&, Output&)> callback) {
    callback(input_blue, out_blue);
    callback(input_yellow, out_yellow);
}

/**
 * @brief Maps an line string type into the respective enum type
 *
 * @param name the name of the line
 * @return SSLLineType
 */
SSLLineType parseLineType(const std::string& name);
/**
 * @brief Maps an arc string type into respective enum type
 *
 * @param name
 * @return SSLArcType
 */
SSLArcType parseArcType(const std::string& name);
/**
 * @brief Maps an referee command into the respective enum type.
 *
 * @param cmd
 * @return SSLCommandType
 */
SSLCommandType parseCommandType(const Referee_Command& cmd);

/**
 * @brief Maps an game event into the respective variant type
 *
 * @param event
 * @return SSLGameEventType
 */
SSLGameEventType parseGameEventType(const GameEvent& event);

SSLWrapperData parseWrapperData(const ssl_vision::SSL_WrapperPacket& packet);
ssl_vision::SSL_WrapperPacket serializeWrapperData(const SSLWrapperData& data);

/**
 * @brief Parses the ssl vision data into our own data format
 *
 * @param frame
 * @return SSLVisionData
 */
SSLVisionData parseVisionData(const ssl_vision::SSL_DetectionFrame& frame);
ssl_vision::SSL_DetectionFrame serializeVisionData(const SSLVisionData& data);

/**
 * @brief Parses the ssl field data into our own data format
 *
 * @param field
 * @return SSLFieldData
 */
SSLFieldData parseFieldData(const ssl_vision::SSL_GeometryFieldSize& field);
ssl_vision::SSL_GeometryFieldSize serializeFieldData(const SSLFieldData& data);

/**
 * @brief  parses the referee data into our own data format
 *
 * @param packet
 * @return SSLGameControllerData
 */
SSLGameControllerData parseRefereeData(const Referee& packet);

}  // namespace luhsoccer::ssl_interface::converter