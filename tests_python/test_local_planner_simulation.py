from tests_python.conftest import baguette, clean_sim_instance, skill_book
import pytest
import time
import math


def test_go_to_point_sim(
    clean_sim_instance: baguette.Baguette, skill_book: baguette.SkillBook
) -> None:
    test_robot = (
        clean_sim_instance.game_data_provider.getWorldModel().getPossibleAllyRobots()[1]
    )
    clean_sim_instance.simulation_interface.teleportRobot(
        [-1.0, 0.0, 0.0], test_robot.getID()
    )

    task_data = baguette.TaskData(test_robot.getID())
    task_data.addPosition(baguette.Position("", 1.0, 0.0, 0.0))

    time.sleep(1)
    result = clean_sim_instance.local_planner_module.getSimulationManager().runSyncSimulation(
        test_robot.getID(),
        skill_book.getSkill(baguette.SkillName.GoToPoint),
        task_data,
        baguette.now(),
    )

    final_transform = result.wm.getTransform(
        test_robot.getID().getFrame(), "", result.end_time
    )
    assert final_transform is not None
    final_position = final_transform.getPositionAndRotation()
    goal_position = [1.0, 0.0, 0.0]
    distance = math.sqrt(
        (final_position[0] - goal_position[0]) ** 2
        + (final_position[1] - goal_position[1]) ** 2
    )

    assert distance < 0.1


def test_mark_enemy_to_ball_sim(
    clean_sim_instance: baguette.Baguette, skill_book: baguette.SkillBook
) -> None:
    test_robot = (
        clean_sim_instance.game_data_provider.getWorldModel().getPossibleAllyRobots()[1]
    )
    enemy_robot = (
        clean_sim_instance.game_data_provider.getWorldModel().getPossibleEnemyRobots()[
            0
        ]
    )

    clean_sim_instance.simulation_interface.teleportRobot(
        [-1.0, 0.0, 0.0], test_robot.getID()
    )

    clean_sim_instance.simulation_interface.teleportRobot(
        [0.0, 2.0, 0.0], enemy_robot.getID()
    )

    task_data = baguette.TaskData(test_robot.getID())
    task_data.addRelatedRobot(enemy_robot.getID())

    time.sleep(1)
    result = clean_sim_instance.local_planner_module.getSimulationManager().runSyncSimulation(
        test_robot.getID(),
        skill_book.getSkill(baguette.SkillName.MarkEnemyToBall),
        task_data,
        baguette.now(),
    )

    final_transform = result.wm.getTransform(
        test_robot.getID().getFrame(), "", result.end_time
    )
    assert final_transform is not None
    final_position = final_transform.getPositionAndRotation()
    # robot has to be on the line from robot to ball

    assert abs(final_position[0]) < 0.05 and 0.0 < final_position[1] < 2.0
