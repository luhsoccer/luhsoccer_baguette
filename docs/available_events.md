<!-- Regex to find Events in code: 'class .*?Event' -->

# List of alll currently available Events

## Core
* StartEvent
* StopEvent

## Game Data Provider
* BallShotEvent
    * ` Eigen::Vector2d old_velocity `
    * ` Eigen::Vector2d new_velocity `
* GameStateChangedEvent
    * ` transform::GameState old_state `
    * ` transform::GameState new_state `
* RealWorldModelUpdatedEvent
    * ` std::shared_ptr<const transform::WorldModel> world_model `
    * ` time::TimePoint timestamp_capture `
    * ` time::TimePoint timestamp_sent `
* RobotRemovedFromFieldEvent
    * ` transform::RobotHandle robot `

## Observer
* DominantTeamChangeEvent
    * ` Team dominant_team `
* ThreatLevelChangedEvent
    * ` RobotIdentifier robot `
    * ` double old_threat_level `
    * ` double new_threat_level `
* RobotMovedEvent
    * ` transform::RobotHandle handle `
    * ` double distance `
    * ` Eigen::Vector2d old_position `
    * ` Eigen::Vector2d new_position ` 
* BallCarrierChangedEvent
    * ` std::optional<transform::RobotHandle> old_carrier `
    * ` std::optional<transform::RobotHandle> new_carrier `

## Timer
* TimerEvent100Hz
* TimerEvent50Hz
* TimerEvent10Hz
* TimerEvent5Hz
* TimerEvent1Hz
* TimerEvent2Sec
* TimerEvent5Sec
* TimerEvent10Sec

## Robot Interface 
* RobotCommandSendEvent
* RobotFeedbackReceivedEvent

## SSL Interface 
* NewVisionDataEvent
* NewFieldDataEvent

