set(MODULE_NAME
    skills
)

add_library(${MODULE_NAME} STATIC
    src/skill.cpp
    
    include/skill_books/skill_tester.hpp
    src/skill_tester.cpp
    
    include/${MODULE_NAME}/skill_book.hpp
    include/${MODULE_NAME}/skill_builder.hpp

    include/skill_books/bod_skill_book.hpp
    src/bod_skill_book/bod_skill_book.cpp

    include/skill_books/game_skill_book.hpp
    src/game_skill_book/game_skill_book.cpp

    include/skill_books/test_skill_book.hpp
    src/test_skill_book/test_skill_book.cpp


    src/game_skill_book/go_to_ball_steps.cpp
    src/game_skill_book/go_to_ball_steps.hpp

    src/game_skill_book/ball_steps.cpp
    src/game_skill_book/ball_steps.hpp

    # add skills here
    # like so: src/bod_skill_book/__template.cpp
    # src/bod_skill_book/go_to_point.cpp
    # include/skill_books/bod_skill_book/go_to_point.hpp
    # src/bod_skill_book/get_ball.cpp
    # include/skill_books/bod_skill_book/get_ball.hpp
    # src/bod_skill_book/go_to_point_aligned.cpp
    # include/skill_books/bod_skill_book/go_to_point_aligned.hpp
    # src/bod_skill_book/halt.cpp
    # include/skill_books/bod_skill_book/halt.hpp
    # src/bod_skill_book/mark_enemy_to_ball.cpp
    # include/skill_books/bod_skill_book/mark_enemy_to_ball.hpp
    # src/bod_skill_book/block_enemy_line_of_sight.cpp
    # include/skill_books/bod_skill_book/block_enemy_line_of_sight.hpp
    # src/bod_skill_book/move_to_penalty_line.cpp
    # include/skill_books/bod_skill_book/move_to_penalty_line.hpp
    # src/bod_skill_book/mark_enemy_to_goal.cpp
    # include/skill_books/bod_skill_book/mark_enemy_to_goal.hpp
    # src/bod_skill_book/wall_at_penalty_area.cpp
    # include/skill_books/bod_skill_book/wall_at_penalty_area.hpp
    # src/bod_skill_book/kick_ball_through_target.cpp
    # include/skill_books/bod_skill_book/kick_ball_through_target.hpp
    # src/bod_skill_book/steal_ball.cpp
    # include/skill_books/bod_skill_book/steal_ball.hpp
    # src/bod_skill_book/intercept_ball.cpp
    # include/skill_books/bod_skill_book/intercept_ball.hpp
    # src/bod_skill_book/intercept_ball_goalie.cpp
    # include/skill_books/bod_skill_book/intercept_ball_goalie.hpp
    # src/bod_skill_book/defend_goal_on_circle.cpp
    # include/skill_books/bod_skill_book/defend_goal_on_circle.hpp
    # src/bod_skill_book/block_goalie_lo_s.cpp
    # include/skill_books/bod_skill_book/block_goalie_lo_s.hpp
    # src/bod_skill_book/reflex_kick.cpp
    # include/skill_books/bod_skill_book/reflex_kick.hpp
    # src/bod_skill_book/prepare_kick.cpp
    # include/skill_books/bod_skill_book/prepare_kick.hpp
    # src/bod_skill_book/defend_goalline.cpp
    # include/skill_books/bod_skill_book/defend_goalline.hpp
    # src/bod_skill_book/wall_at_distance.cpp
    # include/skill_books/bod_skill_book/wall_at_distance.hpp
    # src/bod_skill_book/block_enemy_los_variable.cpp
    # include/skill_books/bod_skill_book/block_enemy_los_variable.hpp
    # src/bod_skill_book/go_to_point_with_heading.cpp
    # include/skill_books/bod_skill_book/go_to_point_with_heading.hpp
    # src/bod_skill_book/move_to_ball_turn_radius.cpp
    # include/skill_books/bod_skill_book/move_to_ball_turn_radius.hpp
    # src/bod_skill_book/drive_to_line.cpp
    # include/skill_books/bod_skill_book/drive_to_line.hpp
    # src/bod_skill_book/okto_skill.cpp
    # include/skill_books/bod_skill_book/okto_skill.hpp
    # src/bod_skill_book/drive_in_ball.cpp
    # include/skill_books/bod_skill_book/drive_in_ball.hpp
    # src/bod_skill_book/backwards_dribbling.cpp
    # include/skill_books/bod_skill_book/backwards_dribbling.hpp
    # src/bod_skill_book/drive_to_line_segment.cpp
    # include/skill_books/bod_skill_book/drive_to_line_segment.hpp
    # src/bod_skill_book/forwards_dribbling.cpp
    # include/skill_books/bod_skill_book/forwards_dribbling.hpp
    # src/bod_skill_book/move_constant.cpp
    # include/skill_books/bod_skill_book/move_constant.hpp
    # src/bod_skill_book/move_constant2.cpp
    # include/skill_books/bod_skill_book/move_constant2.hpp
    # src/bod_skill_book/receive_ball_at_point.cpp
    # include/skill_books/bod_skill_book/receive_ball_at_point.hpp
    # src/bod_skill_book/goalie_los_defend.cpp
    # include/skill_books/bod_skill_book/goalie_los_defend.hpp
    # src/bod_skill_book/goalie_los_defend_w_o_ball.cpp
    # include/skill_books/bod_skill_book/goalie_los_defend_w_o_ball.hpp
    # src/bod_skill_book/intercept_ball_reflex.cpp
    # include/skill_books/bod_skill_book/intercept_ball_reflex.hpp

    src/game_skill_book/go_to_point.cpp
    include/skill_books/game_skill_book/go_to_point.hpp
    src/game_skill_book/halt.cpp
    include/skill_books/game_skill_book/halt.hpp
    src/game_skill_book/go_to_point_with_heading.cpp
    include/skill_books/game_skill_book/go_to_point_with_heading.hpp
    src/game_skill_book/controller_go_to_point.cpp
    include/skill_books/game_skill_book/controller_go_to_point.hpp
    src/game_skill_book/mark_enemy_los_variable.cpp
    include/skill_books/game_skill_book/mark_enemy_los_variable.hpp
    src/game_skill_book/mark_goalie_los.cpp
    include/skill_books/game_skill_book/mark_goalie_los.hpp
    src/game_skill_book/goalie_defend_on_circle.cpp
    include/skill_books/game_skill_book/goalie_defend_on_circle.hpp
    src/game_skill_book/goalie_los_defend.cpp
    include/skill_books/game_skill_book/goalie_los_defend.hpp
    src/game_skill_book/goalie_los_defend_wo_ball.cpp
    include/skill_books/game_skill_book/goalie_los_defend_wo_ball.hpp
    src/game_skill_book/go_to_ball.cpp
    include/skill_books/game_skill_book/go_to_ball.hpp
    src/game_skill_book/get_ball.cpp
    include/skill_books/game_skill_book/get_ball.hpp
    src/game_skill_book/kick_ball.cpp
    include/skill_books/game_skill_book/kick_ball.hpp
    src/game_skill_book/mark_enemy_to_ball.cpp
    include/skill_books/game_skill_book/mark_enemy_to_ball.hpp
    src/game_skill_book/go_to_penalty_line.cpp
    include/skill_books/game_skill_book/go_to_penalty_line.hpp
    src/game_skill_book/wall_at_penalty_area.cpp
    include/skill_books/game_skill_book/wall_at_penalty_area.hpp
    src/game_skill_book/octo_skill.cpp
    include/skill_books/game_skill_book/octo_skill.hpp
    src/game_skill_book/intercept_ball.cpp
    include/skill_books/game_skill_book/intercept_ball.hpp
    src/game_skill_book/kick_reflex.cpp
    include/skill_books/game_skill_book/kick_reflex.hpp
    src/game_skill_book/steal_ball.cpp
    include/skill_books/game_skill_book/steal_ball.hpp
    src/game_skill_book/dribble.cpp
    include/skill_books/game_skill_book/dribble.hpp
    src/game_skill_book/place_ball.cpp
    include/skill_books/game_skill_book/place_ball.hpp
    src/game_skill_book/intercept_ball_at_point.cpp
    include/skill_books/game_skill_book/intercept_ball_at_point.hpp
    # end of skills: DO NOT delete this line

    include/skill_books/skill_library.hpp
    src/skill_library.cpp
)

target_link_libraries(${MODULE_NAME} PUBLIC
    transform
    robot_control
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)