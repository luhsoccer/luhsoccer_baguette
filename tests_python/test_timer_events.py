import baguette_py as baguette
import start_config
import time
import threading


event_100hz = threading.Event()
event_50hz = threading.Event()
event_10hz = threading.Event()
event_1hz = threading.Event()


def custom_setup(instance: baguette.Baguette):
    start_config.setup_clean(instance)

    instance.event_system.registerEventHandler(
        baguette.TimerEvent100Hz, lambda _: event_100hz.set()
    )
    instance.event_system.registerEventHandler(
        baguette.TimerEvent50Hz, lambda _: event_50hz.set()
    )
    instance.event_system.registerEventHandler(
        baguette.TimerEvent10Hz, lambda _: event_10hz.set()
    )
    instance.event_system.registerEventHandler(
        baguette.TimerEvent1Hz, lambda _: event_1hz.set()
    )


@start_config.event_based(custom_setup)
def test_timer_events(baguette_instance: baguette.Baguette):
    time.sleep(2)

    assert event_100hz.is_set()
    assert event_50hz.is_set()
    assert event_10hz.is_set()
    assert event_1hz.is_set()
