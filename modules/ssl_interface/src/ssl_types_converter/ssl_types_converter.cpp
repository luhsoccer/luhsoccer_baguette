#include "ssl_types_converter/ssl_types_converter.hpp"

namespace luhsoccer::ssl_interface::converter {

SSLLineType parseLineType(const std::string& name) {
    static const std::map<std::string, SSLLineType> MAPPINGS{
        {"TopTouchLine", SSLLineType::TOP_TOUCH_LINE},
        {"BottomTouchLine", SSLLineType::BOTTOM_TOUCH_LINE},
        {"LeftGoalLine", SSLLineType::LEFT_GOAL_LINE},
        {"RightGoalLine", SSLLineType::RIGHT_GOAL_LINE},
        {"HalfwayLine", SSLLineType::HALFWAY_LINE},
        {"CenterLine", SSLLineType::CENTER_LINE},
        {"LeftPenaltyStretch", SSLLineType::LEFT_PENALTY_STRETCH},
        {"RightPenaltyStretch", SSLLineType::RIGHT_PENALTY_STRETCH},
        {"LeftFieldLeftPenaltyStretch", SSLLineType::LEFT_FIELD_LEFT_PENALTY_STRETCH},
        {"LeftFieldRightPenaltyStretch", SSLLineType::LEFT_FIELD_RIGHT_PENALTY_STRETCH},
        {"RightFieldRightPenaltyStretch", SSLLineType::RIGHT_FIELD_LEFT_PENALTY_STRETCH},
        {"RightFieldLeftPenaltyStretch", SSLLineType::RIGHT_FIELD_RIGHT_PENALTY_STRETCH},
    };

    auto itr = MAPPINGS.find(name);
    if (itr != MAPPINGS.end()) {
        return itr->second;
    }

    return SSLLineType::UNKNOWN;
}

SSLArcType parseArcType(const std::string& name) {
    static const std::map<std::string, SSLArcType> MAPPINGS{{"CenterCircle", SSLArcType::CENTER_CIRCLE}};

    auto itr = MAPPINGS.find(name);
    if (itr != MAPPINGS.end()) {
        return itr->second;
    }

    return SSLArcType::UNKNOWN;
}

SSLCommandType parseCommandType(const Referee_Command& cmd) {
    switch (cmd) {
        // Currently treat ball placement as stop
        default:
            return static_cast<SSLCommandType>(cmd);
    }
}

SSLGameEventType parseGameEventType(const GameEvent& /*event*/) { return SSLGameEventType::UNKNOWN; }

SSLVisionData parseVisionData(const ssl_vision::SSL_DetectionFrame& frame) {
    SSLVisionData data{};
    // Convert the timepoint in our format
    data.timestamp_sent = time::TimePoint(frame.t_sent());
    data.timestamp_capture = time::TimePoint(frame.t_capture());

    data.frame_number = frame.frame_number();
    data.camera_id = frame.camera_id();

    using SSLRobotList = decltype(frame.robots_blue());
    // Parse both robots
    doForBothColors<SSLRobotList, std::vector<SSLRobotInfo>>(
        frame.robots_blue(), frame.robots_yellow(), data.blue_robots, data.yellow_robots,
        [&](SSLRobotList& robots, std::vector<SSLRobotInfo>& output) {
            for (const auto& robot : robots) {
                // Calculate the transform based on the x and y positions and the rotation of each robot
                const auto transform = Eigen::Translation2d(robot.x() * SCALE_FACTOR, robot.y() * SCALE_FACTOR) *
                                       Eigen::Rotation2Dd(robot.orientation());
                output.emplace_back(
                    SSLRobotInfo{robot.robot_id(), transform, robot.confidence(), {robot.pixel_x(), robot.pixel_y()}});
            }
        });

    // Parse all balls that are visible
    for (const auto& ball : frame.balls()) {
        data.balls.emplace_back(SSLBallInfo{{ball.x() * SCALE_FACTOR, ball.y() * SCALE_FACTOR, ball.z() * SCALE_FACTOR},
                                            ball.confidence(),
                                            {ball.pixel_x(), ball.pixel_y()}});
    }

    return data;
}

ssl_vision::SSL_DetectionFrame serializeVisionData(const SSLVisionData& data) {
    // TODO add more field to allow complete reconstruction
    ssl_vision::SSL_DetectionFrame frame;
    frame.set_t_capture(data.timestamp_capture.asSec());
    frame.set_t_sent(data.timestamp_sent.asSec());
    frame.set_frame_number(data.frame_number);
    frame.set_camera_id(data.camera_id);

    for (const auto& ball : data.balls) {
        auto ball_ptr = std::make_unique<ssl_vision::SSL_DetectionBall>();
        ball_ptr->set_x(static_cast<float>(ball.position.x() / SCALE_FACTOR));
        ball_ptr->set_y(static_cast<float>(ball.position.y() / SCALE_FACTOR));
        ball_ptr->set_z(static_cast<float>(ball.position.z() / SCALE_FACTOR));
        ball_ptr->set_pixel_x(static_cast<float>(ball.pixel_position.x()));
        ball_ptr->set_pixel_y(static_cast<float>(ball.pixel_position.y()));
        ball_ptr->set_confidence(static_cast<float>(ball.confidence));
        frame.mutable_balls()->AddAllocated(ball_ptr.release());
    }

    auto robot_adder = [](const SSLRobotInfo& info, ssl_vision::SSL_DetectionRobot* robot) {
        robot->set_robot_id(info.id);
        robot->set_confidence(static_cast<float>(robot->confidence()));
        robot->set_pixel_x(static_cast<float>(info.pixel_position.x()));
        robot->set_pixel_y(static_cast<float>(info.pixel_position.y()));

        const auto& translation = info.transform.translation();
        robot->set_x(static_cast<float>(translation.x() / SCALE_FACTOR));
        robot->set_y(static_cast<float>(translation.y() / SCALE_FACTOR));

        const auto& rotation = info.transform.rotation();
        robot->set_orientation(static_cast<float>(Eigen::Rotation2Dd(rotation).angle()));
    };

    for (const auto& robot : data.blue_robots) {
        robot_adder(robot, frame.add_robots_blue());
    }

    for (const auto& robot : data.yellow_robots) {
        robot_adder(robot, frame.add_robots_yellow());
    }

    return frame;
}

SSLFieldData parseFieldData(const ssl_vision::SSL_GeometryFieldSize& field) {
    // Read the field length and width
    constexpr int DIV_LENGTH_THRESHOLD = 10.0 / SCALE_FACTOR;
    SSLFieldData data{field.field_length() > DIV_LENGTH_THRESHOLD ? Division::A : Division::B,
                      {field.field_length() * SCALE_FACTOR, field.field_width() * SCALE_FACTOR},
                      field.goal_width() * SCALE_FACTOR,
                      field.goal_depth() * SCALE_FACTOR,
                      field.boundary_width() * SCALE_FACTOR};

    if (field.has_penalty_area_width()) {
        data.penalty_area_width = field.penalty_area_width() * SCALE_FACTOR;
    } else {
        if (data.division == Division::A) {
            data.penalty_area_width = 3.6;
        } else {
            data.penalty_area_width = 2.0;
        }
    }

    if (field.has_penalty_area_depth()) {
        data.penalty_area_depth = field.penalty_area_depth() * SCALE_FACTOR;
    } else {
        if (data.division == Division::A) {
            data.penalty_area_depth = 1.8;
        } else {
            data.penalty_area_depth = 1.0;
        }
    }

    if (field.has_center_circle_radius()) {
        data.center_circle_radius = field.center_circle_radius() * SCALE_FACTOR;
    } else {
        data.center_circle_radius = 0.5;
    }
    if (field.has_line_thickness()) {
        data.line_thickness = field.line_thickness() * SCALE_FACTOR;
    } else {
        data.line_thickness = 0.01;
    }

    if (field.has_goal_center_to_penalty_mark()) {
        data.goal_center_to_penalty_mark = field.goal_center_to_penalty_mark() * SCALE_FACTOR;
    } else {
        if (data.division == Division::A) {
            data.goal_center_to_penalty_mark = 8.0;
        } else {
            data.goal_center_to_penalty_mark = 6.0;
        }
    }

    if (field.has_goal_height()) {
        data.goal_height = field.goal_height() * SCALE_FACTOR;
    } else {
        data.goal_height = 0.16;
    }

    if (field.has_ball_radius()) {
        data.ball_radius = field.ball_radius() * SCALE_FACTOR;
    } else {
        data.ball_radius = 0.043 / 2.0;
    }

    if (field.has_max_robot_radius()) {
        data.max_robot_radius = field.max_robot_radius() * SCALE_FACTOR;
    } else {
        data.max_robot_radius = 0.09;
    }

    // Parse the lines
    for (const auto& line : field.field_lines()) {
        SSLFieldLine parsed_line = SSLFieldLine{line.name(),
                                                parseLineType(line.name()),
                                                {line.p1().x() * SCALE_FACTOR, line.p2().y() * SCALE_FACTOR},
                                                {line.p2().x() * SCALE_FACTOR, line.p2().y() * SCALE_FACTOR},
                                                line.thickness() * SCALE_FACTOR};

        switch (parsed_line.type) {
            case SSLLineType::TOP_TOUCH_LINE:
                data.field_left_top = parsed_line.start_point;
                data.field_right_top = parsed_line.end_point;
                data.field_top_center = {0.0, parsed_line.start_point.y()};
                break;
            case SSLLineType::BOTTOM_TOUCH_LINE:
                data.field_left_bottom = parsed_line.start_point;
                data.field_right_bottom = parsed_line.end_point;
                data.field_bottom_center = {0.0, parsed_line.start_point.y()};
                break;
            case SSLLineType::CENTER_LINE:
                data.field_left_center = parsed_line.start_point;
                data.field_right_center = parsed_line.end_point;
                break;
            case SSLLineType::LEFT_FIELD_LEFT_PENALTY_STRETCH:
                data.penalty_area_left_baseline_bottom = parsed_line.start_point;
                data.penalty_area_left_field_bottom = parsed_line.end_point;
                data.penalty_area_left_field_center = {parsed_line.end_point.x(), 0.0};
                break;
            case SSLLineType::LEFT_FIELD_RIGHT_PENALTY_STRETCH:
                data.penalty_area_left_baseline_top = parsed_line.start_point;
                data.penalty_area_left_field_top = parsed_line.end_point;
                break;
            case SSLLineType::RIGHT_FIELD_LEFT_PENALTY_STRETCH:
                data.penalty_area_right_baseline_bottom = parsed_line.start_point;
                data.penalty_area_right_field_bottom = parsed_line.end_point;
                data.penalty_area_right_field_center = {parsed_line.end_point.x(), 0.0};
                break;
            case SSLLineType::RIGHT_FIELD_RIGHT_PENALTY_STRETCH:
                data.penalty_area_right_baseline_top = parsed_line.start_point;
                data.penalty_area_right_field_top = parsed_line.end_point;
                break;
                // hardcoded Points for the goals - TODO
            case SSLLineType::LEFT_GOAL_LINE:
                // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
                data.goal_left_top = {parsed_line.start_point.x(), 0.5};
                data.goal_left_bottom = {parsed_line.end_point.x(), -0.5};
                break;
            case SSLLineType::RIGHT_GOAL_LINE:
                data.goal_right_top = {parsed_line.start_point.x(), 0.5};
                data.goal_right_bottom = {parsed_line.end_point.x(), -0.5};
                // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
            default:
                break;
        }
        data.lines.emplace_back(std::move(parsed_line));
    }

    // Parse the arcs
    for (const auto& arc : field.field_arcs()) {
        data.arcs.emplace_back(SSLFieldArc{arc.name(),
                                           parseArcType(arc.name()),
                                           {arc.center().x() * SCALE_FACTOR, arc.center().y() * SCALE_FACTOR},
                                           arc.a1(),
                                           arc.a2(),
                                           arc.radius() * SCALE_FACTOR,
                                           arc.thickness() * SCALE_FACTOR});
    }

    return data;
}

static void addLine(ssl_vision::SSL_GeometryFieldSize& size, const std::string& name, Eigen::Vector2d p1,
                    Eigen::Vector2d p2, float thickness) {
    auto line = size.add_field_lines();
    line->set_allocated_name(std::make_unique<std::string>(name).release());
    auto point1 = std::make_unique<ssl_vision::Vector2f>();
    auto point2 = std::make_unique<ssl_vision::Vector2f>();

    point1->set_x(static_cast<float>(p1.x()) / SCALE_FACTOR);
    point1->set_y(static_cast<float>(p1.y()) / SCALE_FACTOR);

    point2->set_x(static_cast<float>(p2.x()) / SCALE_FACTOR);
    point2->set_y(static_cast<float>(p2.y()) / SCALE_FACTOR);

    line->set_allocated_p1(point1.release());
    line->set_allocated_p2(point2.release());
    line->set_thickness(thickness / SCALE_FACTOR);
}

static void addArc(ssl_vision::SSL_GeometryFieldSize& size, const std::string& name, Eigen::Vector2d c, float a1,
                   float a2, float radius, float thickness) {
    auto arc = size.add_field_arcs();
    arc->set_allocated_name(std::make_unique<std::string>(name).release());

    auto center = std::make_unique<ssl_vision::Vector2f>();

    center->set_x(static_cast<float>(c.x()) / SCALE_FACTOR);
    center->set_y(static_cast<float>(c.y()) / SCALE_FACTOR);

    arc->set_allocated_center(center.release());

    arc->set_a1(a1);
    arc->set_a2(a2);
    arc->set_radius(radius / SCALE_FACTOR);
    arc->set_thickness(thickness / SCALE_FACTOR);
}

ssl_vision::SSL_GeometryFieldSize serializeFieldData(const SSLFieldData& data) {
    ssl_vision::SSL_GeometryFieldSize size;

    size.set_field_length(static_cast<int>(data.size.x() / SCALE_FACTOR));
    size.set_field_width(static_cast<int>(data.size.y() / SCALE_FACTOR));
    size.set_goal_width(static_cast<int>(data.goal_width / SCALE_FACTOR));
    size.set_goal_depth(static_cast<int>(data.goal_depth / SCALE_FACTOR));
    size.set_boundary_width(static_cast<int>(data.boundary_width / SCALE_FACTOR));

    for (const auto& line : data.lines) {
        addLine(size, line.name, line.start_point, line.end_point, static_cast<float>(line.thickness));
    }

    for (const auto& arc : data.arcs) {
        addArc(size, arc.name, arc.center, static_cast<float>(arc.start_angle), static_cast<float>(arc.end_angle),
               static_cast<float>(arc.radius), static_cast<float>(arc.thickness));
    }

    return size;
}

SSLGameControllerData parseRefereeData(const Referee& packet) {
    SSLGameControllerData data{};

    data.timestamp_sent = time::Clock::time_point{std::chrono::microseconds{packet.packet_timestamp()}};
    data.timestamp_issued = time::Clock::time_point{std::chrono::microseconds{packet.command_timestamp()}};

    data.command = parseCommandType(packet.command());
    if (packet.has_next_command()) {
        data.next_command = std::make_optional(parseCommandType(packet.next_command()));
    }

    if (packet.has_designated_position()) {
        data.designated_position = {packet.designated_position().x() * SCALE_FACTOR,
                                    packet.designated_position().y() * SCALE_FACTOR};
    }

    doForBothColors<Referee_TeamInfo, SSLTeamInfo>(
        packet.blue(), packet.yellow(), data.blue_team_info, data.yellow_team_info,
        [](const Referee_TeamInfo& info, SSLTeamInfo& info_out) {
            info_out.name = info.name();
            info_out.score = info.score();
            info_out.red_cards = info.red_cards();
            for (const auto& yellow_card_time : info.yellow_card_times()) {
                info_out.yellow_card_times.emplace_back(
                    time::Clock::time_point(std::chrono::microseconds{yellow_card_time}));
            }
            info_out.yellow_cards = info.yellow_cards();
            info_out.timeouts = info.timeouts();
            info_out.timeout_time = time::Clock::time_point(std::chrono::microseconds{info.timeout_time()});
            info_out.goalkeeper = info.goalkeeper();
            if (info.has_foul_counter()) {
                info_out.foul_counter = std::make_optional(info.foul_counter());
            }
            if (info.has_ball_placement_failures()) {
                info_out.ball_placement_failures = std::make_optional(info.ball_placement_failures());
            }
            if (info.has_can_place_ball()) {
                info_out.can_place_ball = std::make_optional(info.can_place_ball());
            }
            if (info.has_max_allowed_bots()) {
                info_out.max_allowed_bots = std::make_optional(info.max_allowed_bots());
            }
            if (info.has_bot_substitution_intent()) {
                info_out.bot_substitution_intent = std::make_optional(info.bot_substitution_intent());
            }
            if (info.has_ball_placement_failures_reached()) {
                info_out.ball_placement_failures_reached = std::make_optional(info.ball_placement_failures_reached());
            }
        });

    return data;
}

SSLWrapperData parseWrapperData(const ssl_vision::SSL_WrapperPacket& packet) {
    SSLWrapperData data;
    if (packet.has_detection()) {
        data.vision = parseVisionData(packet.detection());
    }

    if (packet.has_geometry()) {
        const auto& geometry = packet.geometry();

        if (geometry.has_field()) {
            data.field = parseFieldData(geometry.field());
        }
    }

    return data;
}

ssl_vision::SSL_WrapperPacket serializeWrapperData(const SSLWrapperData& data) {
    ssl_vision::SSL_WrapperPacket packet;
    if (data.vision) {
        auto vision_data = std::make_unique<ssl_vision::SSL_DetectionFrame>(serializeVisionData(data.vision.value()));
        packet.set_allocated_detection(vision_data.release());
    }

    if (data.field) {
        auto geometry_data = std::make_unique<ssl_vision::SSL_GeometryData>();
        auto field_size = std::make_unique<ssl_vision::SSL_GeometryFieldSize>(serializeFieldData(data.field.value()));
        geometry_data->set_allocated_field(field_size.release());
        packet.set_allocated_geometry(geometry_data.release());
    }

    return packet;
}

}  // namespace luhsoccer::ssl_interface::converter