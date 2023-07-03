from tests_python.conftest import baguette_instance, baguette


def test_time(baguette_instance: baguette.Baguette):
    now = baguette.now()
    print(f"{now} {now.asSec()} {now.asNSec()}")
