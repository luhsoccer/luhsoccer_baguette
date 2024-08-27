import baguette_py as baguette
import functools
import threading

import numpy as np


def event_based(setup_callback):
    def on_start(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            if len(kwargs) == 0:
                raise ValueError("You must provide a 'baguette_instance' argument")

            instance = kwargs.get(
                "baguette_instance",
            )

            if not isinstance(instance, baguette.Baguette):
                raise ValueError("You must provide a 'baguette_instance' argument")

            event = threading.Event()
            exception: Exception | None = None

            def on_start(_: baguette.StartEvent):
                event.set()
                nonlocal exception
                try:
                    func(*args, **kwargs)
                except Exception as e:
                    exception = e
                instance.stop()

            def setup():
                instance.event_system.registerEventHandler(
                    baguette.StartEvent, on_start
                )

                nonlocal exception
                try:
                    setup_callback(instance)
                except Exception as e:
                    exception = e

            instance.start(setup)
            assert event.is_set()
            if exception is not None:
                raise exception  # type: ignore (was checked before)

        return wrapper

    return on_start


def setup_clean(baguette_instance: baguette.Baguette):
    pass


def setup_with_sim(instance: baguette.Baguette):
    instance.ssl_interface.setVisionDataSource(baguette.VisionDataSource.Simulation)

    instance.ssl_interface.setGameControllerDataSource(
        baguette.GamecontrollerDataSource.Network
    )

    instance.robot_interface.setConnectionType(baguette.RobotConnection.Simulation)

    instance.simulation_interface.switchConnector(
        baguette.SimulationConnectorType.ErforceSimulation
    )


def setup_with_clean_sim(instance: baguette.Baguette):
    instance.ssl_interface.setVisionDataSource(baguette.VisionDataSource.Simulation)
    instance.ssl_interface.setGameControllerDataSource(
        baguette.GamecontrollerDataSource.Network
    )
    instance.robot_interface.setConnectionType(baguette.RobotConnection.Simulation)
    instance.simulation_interface.switchConnector(
        baguette.SimulationConnectorType.ErforceSimulation
    )
    wm = instance.game_data_provider.getWorldModel()
    for robot in wm.getPossibleAllyRobots():
        instance.simulation_interface.teleportRobot(
            np.array([0.0, 0.0, 0.0]),
            robot.getID(),
            np.array([0.0, 0.0, 0.0]),
            False,
        )
    instance.robot_control_module.cancelTask(robot.getID())
    for robot in wm.getPossibleEnemyRobots():
        instance.simulation_interface.teleportRobot(
            np.array([0.0, 0.0, 0.0]),
            robot.getID(),
            np.array([0.0, 0.0, 0.0]),
            False,
        )
        instance.robot_control_module.cancelTask(robot.getID())
    instance.simulation_interface.teleportBall(np.array([0.0, 0.0, 0.0]))
