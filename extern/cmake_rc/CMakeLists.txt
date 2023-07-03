include("CMakeRC.cmake")

cmrc_add_resource_library(

    config_provider_resources
    ALIAS config_provider::cfgs

    NAMESPACE config_provider

    # ADD CMAKE_RC TOML FILES HERE: 
    configs/test_file.toml
    configs/local_planner_components.toml
    configs/real_local_planner_components.toml
    configs/game.toml
    configs/yellow_game.toml
    configs/skills.toml
    configs/game_data_provider_shared.toml
    configs/observer_config_file.toml
)

cmrc_add_resource_library(
    luhviz_ressources

    ALIAS luhviz::rc
    NAMESPACE luhviz

    res/fonts/DroidSans.ttf
    res/fonts/Roboto-Regular.ttf
    res/fonts/RobotoMono-Regular.ttf
    res/fonts/Arimo-Regular.ttf
    res/fonts/Cousine-Regular.ttf
    res/fonts/Roboto-Medium.ttf
    res/fonts/JetBrainsMono-Regular.ttf
    res/fonts/VeraMono.ttf
    res/fonts/DejaVuSansMono.ttf

    res/images/robot_icon.png
    res/images/splashscreen.png
    res/images/cursor_icons/teleport_cursor.png
    res/images/cursor_icons/setpoint_cursor.png
    res/images/cursor_icons/setdir_cursor.png
    res/images/cursor_icons/execskill_cursor.png
    res/images/cursor_icons/ballslingshot_cursor.png
    res/images/status_icons/controler_connected.png
    res/images/status_icons/controler_disconnected.png
    res/images/status_icons/controler_input.png
    res/images/player_icons/player_play.png
    res/images/player_icons/player_pause.png
    res/images/player_icons/player_stop.png
    res/images/player_icons/player_rewind.png
    res/images/controller_icons/gamepad.png
    res/images/reset_icon.png
    res/images/luhbots_logo_100.png
    res/images/luhbots_logo_80.png
    res/images/luhbots_logo_50.png
    res/images/luhbots_logo_30.png

    res/obj/arrow.bmp
    res/obj/arrow.obj
    res/obj/ball.bmp
    res/obj/ball.obj
    res/obj/cone.obj
    res/obj/cube.obj
    res/obj/cylinder.obj
    res/obj/frame.bmp
    res/obj/frame.obj
    res/obj/goalborders.obj
    res/obj/plane.obj
    res/obj/soccerrobot.obj
    res/obj/sphere.obj
    res/obj/suzanne.obj
    res/obj/torus.obj

    res/shader/simpleTextureFragmentShader.glsl
    res/shader/simpleTextureVertexShader.glsl
    res/shader/vertexShader.glsl
    res/shader/fragmentShader.glsl

    res/layout/luhviz_default.ini

    res/cat/cat0.txt
    res/cat/cat1.txt
    res/cat/cat2.txt
    res/cat/cat3.txt
    res/cat/cat4.txt
    res/cat/cat5.txt
    res/cat/cat6.txt
    res/cat/cat7.txt

    # team logos
    res/team_logos/luhbots.png
    res/team_logos/unknown.png
    res/team_logos/ais.png
    res/team_logos/cmus.png
    res/team_logos/er-force.png
    res/team_logos/immortals.png
    res/team_logos/kiks.png
    res/team_logos/mct\ susano\ logics.png
    res/team_logos/mrl.png
    res/team_logos/namec.png
    res/team_logos/neuislanders.png
    res/team_logos/op-amp.png
    res/team_logos/rione.png
    res/team_logos/robocin.png
    res/team_logos/robodragons.png
    res/team_logos/robofei.png
    res/team_logos/roboime.png
    res/team_logos/robojackets.png
    res/team_logos/roboteam\ twente.png
    res/team_logos/src.png
    res/team_logos/tigers\ mannheim.png
    res/team_logos/ubc\ thunderbots.png
    res/team_logos/ultron.png
    res/team_logos/umass\ minutebots.png
    res/team_logos/uroborus.png
    res/team_logos/zjunlict.png
)
