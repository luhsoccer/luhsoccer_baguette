import pytest
from tests_python.conftest import skill_book, clean_sim_instance, baguette
import time
import math
import random


@pytest.mark.usefixtures("clean_sim_instance", "skill_book")
def test_kick_through_target_random_direction(
    clean_sim_instance: baguette.Baguette, skill_book: baguette.SkillBook
) -> None:
    wm = clean_sim_instance.game_data_provider.getWorldModel()
    test_robot = wm.getPossibleAllyRobots()[1]
    clean_sim_instance.simulation_interface.teleportRobot(
        [-1.0, 0.0, 0.0], test_robot.getID()
    )

    random_angle = random.random() * math.pi * 2
    distance = 1.0
    goal_position = [
        math.cos(random_angle) * distance,
        math.sin(random_angle) * distance,
    ]

    task_data = baguette.TaskData(test_robot.getID())
    task_data.addPosition(
        baguette.Position("", goal_position[0], goal_position[1], 0.0)
    )
    task_data.addDouble(6.0)

    time.sleep(1)

    clean_sim_instance.local_planner_module.setTask(
        skill_book.getSkill(baguette.SkillName.KickBallThroughTarget), task_data
    )

    run = True
    start = time.time()
    timeout = 5.0
    while run:
        position = wm.getBallPosition()
        angle_diff = abs(math.atan2(position[1], position[0]) - random_angle)
        distance_from_start = math.sqrt(position[0] ** 2 + position[1] ** 2)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
            angle_diff *= -1
        if angle_diff < 0.1 and distance_from_start > 1.0:
            run = False

        now = time.time()
        if now - start > timeout:
            run = False
            pytest.fail()
