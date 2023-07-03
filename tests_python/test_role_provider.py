from tests_python.conftest import (
    baguette,
    baguette_instance,
    role_manager,
    task_manager,
    simulation_interface,
)

import pytest
import time
import typing


class CustomRoleProvider(baguette.RoleProvider):
    def getPossibleRoles(self):
        return ["unknown"]

    def provideRoles(self, allies, enemies, observer):
        roles = []
        for robot in allies:
            print(f"Got ally robot: {robot.getID()}, Pos: {robot.getPosition()}.")
            roles.append((robot, "unknown"))
        print(
            f"Ball: {baguette.getInstance().game_data_provider.getWorldModel().getBallPosition()} Observer info: {observer.getBallGoalProbability()}. Roles {roles}"
        )

        return roles


def test_role_provider(
    baguette_instance: baguette.Baguette,
    simulation_interface: baguette.Baguette,
    task_manager: baguette.Baguette,
    role_manager: baguette.Baguette,
) -> None:
    custom_role_provider = CustomRoleProvider()
    role_manager.setRoleProvider(custom_role_provider)

    def my_role_callback(robots, observer, wm):
        skills: typing.Dict[
            baguette.RobotIdentifier,
            typing.Tuple[baguette.SkillName, baguette.TaskData],
        ] = {}
        for robot in robots:
            print(f"Provide task for robot {robot}")
            td = baguette.TaskData(robot.getID())
            td.addPosition(baguette.Position("", 2.0, 2.0, 0.0))
            # td.required_positions.append(baguette.Position("", 2.0, 2.0, 0.0))
            skills[robot.getID()] = (baguette.SkillName.GoToPoint, td)

        print("sleep")
        time.sleep(2)

        return skills

    task_manager.addRoleCallback("unknown", my_role_callback)

    baguette_instance.start()
    # simulation_interface.switchConnector(baguette.SimulationConnectorType.TestSimulation)

    # This is sadly not possible. See https://github.com/pybind/pybind11/issues/1333 for more details
    # role_manager().setRoleProvider(CustomRoleProvider())

    time.sleep(10)
