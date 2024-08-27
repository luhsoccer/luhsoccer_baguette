import baguette_py as baguette
import start_config
import time
import math
import numpy as np
import random


@start_config.event_based(start_config.setup_with_clean_sim)
def test_go_to_point(
    baguette_instance: baguette.Baguette, skill_library: baguette.SkillLibrary
):
    test_robot = (
        baguette_instance.game_data_provider.getWorldModel().getPossibleAllyRobots()[1]
    )
    baguette_instance.simulation_interface.teleportRobot(
        np.array([-1.0, 0.0, 0.0]), test_robot.getID()
    )

    task_data = baguette.TaskData(test_robot.getID())
    task_data.addPosition(baguette.Position("", 1.0, 0.0, 0.0))

    time.sleep(1)
    baguette_instance.robot_control_module.setTask(
        skill_library.getSkill(baguette.GameSkillNames.GoToPoint), task_data
    )

    run: bool = True
    start = time.time()
    goal = [1.0, 0.0, 0.0]
    timeout = 5.0
    while run:
        position = test_robot.getPosition()
        distance_to_goal = math.sqrt(
            (position[0] - goal[0]) ** 2 + (position[1] - goal[1]) ** 2
        )
        if distance_to_goal < 0.07:
            run = False
        now = time.time()
        if now - start > timeout:
            run = False
            raise Exception("Skill took to long to reach target position")


@start_config.event_based(start_config.setup_with_clean_sim)
def test_go_to_point_wit_obstacles(
    baguette_instance: baguette.Baguette, skill_library: baguette.SkillLibrary
):
    test_robot = (
        baguette_instance.game_data_provider.getWorldModel().getPossibleAllyRobots()[1]
    )
    time.sleep(1)

    for i in range(10):
        baguette_instance.simulation_interface.teleportRobot(
            np.array([-3.0, 0.0, 0.0]), test_robot.getID()
        )

        # randomly place enemy robots
        MAX_X = 2.5
        MAX_Y = 1.0
        possible_enemy_robots = baguette_instance.game_data_provider.getWorldModel().getPossibleEnemyRobots()
        for x in range(6):
            random_x = (random.random() * 2 * MAX_X) - MAX_X
            random_y = (random.random() * 2 * MAX_Y) - MAX_Y
            baguette_instance.simulation_interface.teleportRobot(
                np.array([random_x, random_y, 0.0]), possible_enemy_robots[x].getID()
            )

        task_data = baguette.TaskData(test_robot.getID())
        task_data.addPosition(baguette.Position("", 3.0, 0.0, 0.0))
        time.sleep(1)

        baguette_instance.robot_control_module.setTask(
            skill_library.getSkill(baguette.GameSkillNames.GoToPoint), task_data
        )

        run: bool = True
        start = time.time()
        goal = [3.0, 0.0, 0.0]
        timeout = 10.0
        while run:
            position = test_robot.getPosition()
            distance_to_goal = math.sqrt(
                (position[0] - goal[0]) ** 2 + (position[1] - goal[1]) ** 2
            )
            if distance_to_goal < 0.07:
                run = False
            now = time.time()
            if now - start > timeout:
                run = False
                raise Exception("Skill took to long to reach target position")
