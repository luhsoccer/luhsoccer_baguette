import pytest
from tests_python.conftest import skill_book, clean_sim_instance, baguette
import random
import time
import math


@pytest.mark.usefixtures("clean_sim_instance", "skill_book")
def test_go_to_point(
    clean_sim_instance: baguette.Baguette, skill_book: baguette.SkillBook
):
    test_robot = (
        clean_sim_instance.game_data_provider.getWorldModel().getPossibleAllyRobots()[1]
    )
    clean_sim_instance.simulation_interface.teleportRobot(
        [-1.0, 0.0, 0.0], test_robot.getID()
    )

    task_data = baguette.TaskData(test_robot.getID())
    task_data.addPosition(baguette.Position("", 1.0, 0.0, 0.0))

    time.sleep(1)
    clean_sim_instance.local_planner_module.setTask(
        skill_book.getSkill(baguette.SkillName.GoToPoint), task_data
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
            pytest.fail()


@pytest.mark.usefixtures("clean_sim_instance", "skill_book")
def test_go_to_point_wit_obstacles(
    clean_sim_instance: baguette.Baguette, skill_book: baguette.SkillBook
):
    test_robot = (
        clean_sim_instance.game_data_provider.getWorldModel().getPossibleAllyRobots()[1]
    )
    time.sleep(1)

    for i in range(10):
        clean_sim_instance.simulation_interface.teleportRobot(
            [-3.0, 0.0, 0.0], test_robot.getID()
        )

        # randomly place enemy robots
        MAX_X = 2.5
        MAX_Y = 1.0
        possible_enemy_robots = (
            clean_sim_instance.game_data_provider.getWorldModel().getPossibleEnemyRobots()
        )
        for x in range(6):
            random_x = (random.random() * 2 * MAX_X) - MAX_X
            random_y = (random.random() * 2 * MAX_Y) - MAX_Y
            clean_sim_instance.simulation_interface.teleportRobot(
                [random_x, random_y, 0.0], possible_enemy_robots[x].getID()
            )

        task_data = baguette.TaskData(test_robot.getID())
        task_data.addPosition(baguette.Position("", 3.0, 0.0, 0.0))
        time.sleep(1)

        clean_sim_instance.local_planner_module.setTask(
            skill_book.getSkill(baguette.SkillName.GoToPoint), task_data
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
                pytest.fail()
