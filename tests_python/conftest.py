import _baguette_py as baguette
import pytest
import time


@pytest.fixture
def baguette_instance() -> baguette.Baguette:
    return baguette.getInstance()


@pytest.fixture
def sim_instance(baguette_instance: baguette.Baguette) -> baguette.Baguette:
    baguette_instance.start()
    baguette_instance.ssl_interface.setVisionDataSource(
        baguette.VisionDataSource.Simulation
    )
    baguette_instance.ssl_interface.setGameControllerDataSource(
        baguette.GamecontrollerDataSource.Network
    )
    baguette_instance.robot_interface.setConnectionType(
        baguette.RobotConnection.Simulation
    )
    baguette_instance.simulation_interface.switchConnector(
        baguette.SimulationConnectorType.ErForceSimulation
    )
    return baguette_instance


@pytest.fixture
def clean_sim_instance(sim_instance: baguette.Baguette) -> baguette.Baguette:
    wm = sim_instance.game_data_provider.getWorldModel()
    for robot in wm.getPossibleAllyRobots():
        sim_instance.simulation_interface.teleportRobot(
            [0.0, 0.0, 0.0], robot.getID(), [0.0, 0.0, 0.0], False
        )
        sim_instance.local_planner_module.cancelTask(robot.getID())
    for robot in wm.getPossibleEnemyRobots():
        sim_instance.simulation_interface.teleportRobot(
            [0.0, 0.0, 0.0], robot.getID(), [0.0, 0.0, 0.0], False
        )
        sim_instance.local_planner_module.cancelTask(robot.getID())

    sim_instance.simulation_interface.teleportBall([0.0, 0.0, 0.0])
    return sim_instance


@pytest.fixture
def ssl_interface(baguette_instance: baguette.Baguette) -> baguette.SSLInterface:
    return baguette_instance.ssl_interface


@pytest.fixture
def role_manager(baguette_instance: baguette.Baguette) -> baguette.RoleManager:
    return baguette_instance.role_manager


@pytest.fixture
def game_data_provider(
    baguette_instance: baguette.Baguette,
) -> baguette.GameDataProvider:
    return baguette_instance.game_data_provider


@pytest.fixture
def marker_service(baguette_instance: baguette.Baguette) -> baguette.MarkerService:
    return baguette_instance.marker_service


@pytest.fixture
def skill_book(baguette_instance: baguette.Baguette) -> baguette.SkillBook:
    return baguette_instance.skill_book


@pytest.fixture
def task_manager(baguette_instance: baguette.Baguette) -> baguette.TaskManager:
    return baguette_instance.task_manager


@pytest.fixture
def simulation_interface(
    baguette_instance: baguette.Baguette,
) -> baguette.SimulationInterface:
    return baguette_instance.simulation_interface
