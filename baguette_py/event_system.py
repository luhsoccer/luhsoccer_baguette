from collections.abc import Callable
from typing import TypeVar
import sys
import traceback

from ._baguette_py import (
    Event,
    NativeEventSystem,
    EventQueue,
    PyEvent,
    StopEvent,
    Logger,
)

EventType = TypeVar("EventType", bound=Event)
EventCallback = Callable[[EventType], None]
EventPredicate = Callable[[EventType], bool]


class EventHandler:
    def __init__(
        self, callback: EventCallback, predicate: EventPredicate | None
    ) -> None:
        self.callback = callback
        self.predicate = predicate


class EventSystem:
    def __init__(self, native_event_system: NativeEventSystem) -> None:
        self._native_event_system: NativeEventSystem = native_event_system
        self._event_queue = EventQueue()
        self._logger = Logger("EventSystem")
        self._event_handlers: dict[type, list[EventHandler]] = {}

        # Enable receiving for PyEvent, since it's not a real event type and only fired from python, which can't be pre filtered
        self._event_queue.enableReceivingFor(self._native_event_system, PyEvent)
        # Enable receiving for StopEvent, since we need to know when to stop the event loop
        self._event_queue.enableReceivingFor(self._native_event_system, StopEvent)

    def fireEvent(self, event: Event) -> None:
        if type(event) == PyEvent:
            # PyEvent can be enqueued directly since baguette can't (and shouldn't) use them anyway
            self._event_queue.enqueue(event)
        else:
            self._native_event_system.fireEvent(event)

    def registerEventHandler(
        self,
        event_type: type[EventType],
        callback: EventCallback,
        predicate: EventPredicate | None = None,
    ) -> None:
        if event_type not in self._event_handlers:
            # PyEvent are always registered in the queue since baguette can't differentiate between PyEvents
            if not issubclass(event_type, PyEvent):
                self._event_queue.enableReceivingFor(
                    self._native_event_system, event_type
                )

            self._event_handlers[event_type] = []

        self._event_handlers[event_type].append(EventHandler(callback, predicate))

    def event_loop(self) -> None:
        while True:
            event = self._event_queue.pop()

            if type(event) in self._event_handlers:
                for handler in self._event_handlers[type(event)]:
                    if handler.predicate is None or handler.predicate(event):
                        try:
                            handler.callback(event)
                        except Exception as _:
                            self._logger.error(
                                f"Exception occurred while handling event {event} with callback {handler.callback}.\n{sys.exc_info()[0](traceback.format_exc())}"
                            )

            if type(event) == StopEvent:
                # Enqueue the event again so that the other threads can also stop
                self._logger.debug("Stopping event loop")
                self._event_queue.enqueue(event)
                break
