set(MODULE_NAME
    skills
)

add_library(${MODULE_NAME} STATIC
    src/skill.cpp
    include/skill_books/bod_skill_book.hpp
    include/skill_books/bod_skill_book/get_ball.hpp
    include/skill_books/bod_skill_book/go_to_point.hpp


    include/skill_books/skill_tester.hpp
    src/skill_tester.cpp
    
    include/${MODULE_NAME}/skill_book.hpp
    include/${MODULE_NAME}/skill_builder.hpp

    src/bod_skill_book/bod_skill_book.cpp

    # add skills here
    # like so: src/bod_skill_book/__template.cpp
    src/bod_skill_book/go_to_point.cpp
    src/bod_skill_book/get_ball.cpp
    src/bod_skill_book/drive_by_side.cpp
    include/skill_books/bod_skill_book/drive_by_side.hpp
    src/bod_skill_book/go_to_point_aligned.cpp
    include/skill_books/bod_skill_book/go_to_point_aligned.hpp
    src/bod_skill_book/halt.cpp
    include/skill_books/bod_skill_book/halt.hpp
    src/bod_skill_book/mark_enemy_to_ball.cpp
    include/skill_books/bod_skill_book/mark_enemy_to_ball.hpp
    src/bod_skill_book/block_enemy_line_of_sight.cpp
    include/skill_books/bod_skill_book/block_enemy_line_of_sight.hpp
    src/bod_skill_book/move_to_penalty_line.cpp
    include/skill_books/bod_skill_book/move_to_penalty_line.hpp
    src/bod_skill_book/mark_enemy_to_goal.cpp
    include/skill_books/bod_skill_book/mark_enemy_to_goal.hpp
    src/bod_skill_book/wall_at_penalty_area.cpp
    include/skill_books/bod_skill_book/wall_at_penalty_area.hpp
    src/bod_skill_book/kick_ball_to_target.cpp
    include/skill_books/bod_skill_book/kick_ball_to_target.hpp
    src/bod_skill_book/kick_ball_through_target.cpp
    include/skill_books/bod_skill_book/kick_ball_through_target.hpp
    src/bod_skill_book/steal_ball.cpp
    include/skill_books/bod_skill_book/steal_ball.hpp
    src/bod_skill_book/receive_ball.cpp
    include/skill_books/bod_skill_book/receive_ball.hpp
    src/bod_skill_book/defend_goal_on_circle.cpp
    include/skill_books/bod_skill_book/defend_goal_on_circle.hpp
    src/bod_skill_book/block_goalie_lo_s.cpp
    include/skill_books/bod_skill_book/block_goalie_lo_s.hpp
    src/bod_skill_book/pass_ball_to_robot.cpp
    include/skill_books/bod_skill_book/pass_ball_to_robot.hpp
    src/bod_skill_book/reflex_kick.cpp
    include/skill_books/bod_skill_book/reflex_kick.hpp
    src/bod_skill_book/prepare_kick.cpp
    include/skill_books/bod_skill_book/prepare_kick.hpp
    src/bod_skill_book/defend_goalline.cpp
    include/skill_books/bod_skill_book/defend_goalline.hpp
    src/bod_skill_book/wall_at_distance.cpp
    include/skill_books/bod_skill_book/wall_at_distance.hpp
    src/bod_skill_book/block_enemy_los_variable.cpp
    include/skill_books/bod_skill_book/block_enemy_los_variable.hpp
    src/bod_skill_book/go_to_point_with_heading.cpp
    include/skill_books/bod_skill_book/go_to_point_with_heading.hpp
    src/bod_skill_book/move_to_ball_turn_radius.cpp
    include/skill_books/bod_skill_book/move_to_ball_turn_radius.hpp
    src/bod_skill_book/drive_to_line.cpp
    include/skill_books/bod_skill_book/drive_to_line.hpp
    src/bod_skill_book/kick_ball_through_target_preposition.cpp
    include/skill_books/bod_skill_book/kick_ball_through_target_preposition.hpp
    src/bod_skill_book/kick_ball_through_target_original.cpp
    include/skill_books/bod_skill_book/kick_ball_through_target_original.hpp
    src/bod_skill_book/kick_ball_through_target_direct.cpp
    include/skill_books/bod_skill_book/kick_ball_through_target_direct.hpp
    src/bod_skill_book/run_free.cpp
    include/skill_books/bod_skill_book/run_free.hpp
    src/bod_skill_book/okto_skill.cpp
    include/skill_books/bod_skill_book/okto_skill.hpp
    src/bod_skill_book/drive_in_ball.cpp
    include/skill_books/bod_skill_book/drive_in_ball.hpp
    src/bod_skill_book/go_to_point_original.cpp
    include/skill_books/bod_skill_book/go_to_point_original.hpp
    src/bod_skill_book/go_to_point_with_heading_original.cpp
    include/skill_books/bod_skill_book/go_to_point_with_heading_original.hpp
    src/bod_skill_book/go_to_point_dribbler.cpp
    include/skill_books/bod_skill_book/go_to_point_dribbler.hpp
    src/bod_skill_book/backwards_dribbling.cpp
    include/skill_books/bod_skill_book/backwards_dribbling.hpp
    src/bod_skill_book/drive_to_line_segment.cpp
    include/skill_books/bod_skill_book/drive_to_line_segment.hpp
    src/bod_skill_book/go_to_point_intercept.cpp
    include/skill_books/bod_skill_book/go_to_point_intercept.hpp
    src/bod_skill_book/forwards_dribbling.cpp
    include/skill_books/bod_skill_book/forwards_dribbling.hpp
    # end of skills: DO NOT delete this line

)

target_link_libraries(${MODULE_NAME} PUBLIC
    transform
    local_planner
    local_planner_components
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)