#include "serial/serial.h"
#include "luhsoccer_robot_interface.pb.h"
#include "logger/logger.hpp"
#include "packets.hpp"

namespace luhsoccer::robot_interface {

struct SerialToRobotPacket {
    unsigned int packet_type : 2 = 1;
    unsigned int robot_id : 4 = 0;
    unsigned int dribbler_strength : 2 = 0;
    signed int velocity_x : 16 = 0;
    signed int velocity_y : 16 = 0;
    signed int rotation : 16 = 0;
    bool lift_kick : 1 = false;
    unsigned int kicker_strength : 7 = 0;
    unsigned int kicker_voltage = 0;
};

struct SerialFromRobotPacket {
    unsigned int packet_type : 2 = 1;
    unsigned int robot_id : 4 = 0;
    unsigned int error : 2 = 0;
    unsigned int battery_voltage : 7 = 0;
    bool has_ball_in_dribbler : 1 = false;
    unsigned int kicker_voltage : 8 = 0;
    signed int delta_x : 8 = 0;
    signed int delta_y : 8 = 0;
    signed int delta_rotation : 8 = 0;
};

constexpr inline size_t TO_ROBOT_PACKET_SIZE = 9;
using ToRobotPacketBuffer = std::array<char, TO_ROBOT_PACKET_SIZE>;

constexpr inline size_t FROM_ROBOT_PACKET_SIZE = 6;
using FromRobotPacketBuffer = std::array<unsigned char, FROM_ROBOT_PACKET_SIZE>;

class SerialConnection {
   public:
    std::vector<RobotFeedbackWrapper> send(const std::string& data);
    void stop();

   private:
    RobotFeedbackWrapper fromSerialPacket(const FromRobotPacketBuffer& buffer);

    static inline constexpr int BASE_STATION_V1_BAUDRATE = 115200;
    static inline constexpr int SERIAL_PORT_TIMEOUT_MS = 1000;
    serial::Serial port{"",
                        BASE_STATION_V1_BAUDRATE,
                        serial::Timeout::simpleTimeout(SERIAL_PORT_TIMEOUT_MS),
                        serial::eightbits,
                        serial::parity_none,
                        serial::stopbits_one,
                        serial::flowcontrol_none};
    logger::Logger logger{"serial_connection"};
};

class SerialPacketBuilder : public PacketBuilder {
   public:
    SerialPacketBuilder(SerialConnection&);
    void addMessage(const RobotCommandWrapper& cmd) override;
    std::vector<RobotFeedbackWrapper> buildAndSend() override;

   private:
    SerialConnection& connection;
    std::vector<SerialToRobotPacket> packets;
};

};  // namespace luhsoccer::robot_interface