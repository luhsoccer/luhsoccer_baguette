from ._baguette_py import (
    NativeBaguette,
    GameDataProvider,
    RobotControlModule,
    MarkerService,
    RobotInterface,
    SimulationInterface,
    SkillBook,
    SkillLibrary,
    SSLInterface,
    TaskManager,
)

from .event_system import EventSystem
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor


class Baguette:
    _init = False
    _create_key = object()
    _baguette_instance: "Baguette | None" = None

    def __init__(self, create_key) -> None:
        """
        Should not be called directly. Use getInstance instead.
        """
        assert (
            create_key == Baguette._create_key
        ), "Baguette should not be created directly"

        if Baguette._init:
            raise RuntimeError("Baguette already initialized")
        self._instance = NativeBaguette.getInstance()
        self._event_system = EventSystem(self._instance.native_event_system)
        self._threads = 4
        self._executor = ThreadPoolExecutor(self._threads)
        self._started = False
        Baguette._init = True

    @property
    def game_data_provider(self) -> GameDataProvider:
        return self._instance.game_data_provider

    @property
    def robot_control_module(self) -> RobotControlModule:
        return self._instance.robot_control_module

    @property
    def marker_service(self) -> MarkerService:
        return self._instance.marker_service

    @property
    def robot_interface(self) -> RobotInterface:
        return self._instance.robot_interface

    @property
    def simulation_interface(self) -> SimulationInterface:
        return self._instance.simulation_interface

    @property
    def skill_book(self) -> SkillBook:
        return self._instance.skill_book

    @property
    def skill_library(self) -> SkillLibrary:
        return self._instance.skill_lib

    @property
    def ssl_interface(self) -> SSLInterface:
        return self._instance.ssl_interface

    @property
    def task_manager(self) -> TaskManager:
        return self._instance.task_manager

    @property
    def event_system(self) -> EventSystem:
        return self._event_system

    @staticmethod
    def stop() -> None:
        NativeBaguette.stop()

    @classmethod
    def getInstance(cls) -> "Baguette":
        if cls._baguette_instance is not None:
            return cls._baguette_instance
        cls._baguette_instance = Baguette(cls._create_key)
        return cls._baguette_instance

    def start(self, setup_function: Callable[[], None]) -> None:  # noqa: FBT001, FBT002
        if self._started:
            raise RuntimeError("Baguette already started")

        self._started = True

        def start_callback() -> None:
            setup_function()
            for _ in range(self._threads):
                self._executor.submit(self._event_system.event_loop)

        return self._instance.start(start_callback)
