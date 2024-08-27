# Event System {#event_system_module}

## Register an event handler

An event handler must be register in a `setup` function of a module.

Example of register an event handler to the 1Hz event (A timer which will be executed once per second):

```cpp
void GameDataProvider::setup(event_system::EventSystem& system) {
     system.registerEventHandler<event_system::TimerEvent1Hz>(
        [this](const event_system::EventContext<event_system::TimerEvent1Hz>&) {
            logger.info("Test");
        }
    );
}
```

This code will print `Test` once per second
