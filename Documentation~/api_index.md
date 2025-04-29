---
uid: api-index
---

# Unity Vehicles API reference

This page contains an overview of some key APIs of the Character Controller package.

| **Authorings** | **Description**|
| :--- | :--- |
| [VehicleAuthoring](xref:Unity.Vehicles.VehicleAuthoring) | Authoring component for vehicles. |
| [WheelAuthoring](xref:Unity.Vehicles.WheelAuthoring) | Authoring component for wheels. |
| [VehicleControlAuthoring](xref:Unity.Vehicles.VehicleControlAuthoring) | Authoring component for vehicle user control. |
| [ControllableWheelAuthoring](xref:Unity.Vehicles.ControllableWheelAuthoring) | Authoring component for wheel user control. |

| **Components** | **Description**|
| :--- | :--- |
| [Vehicle](xref:Unity.Vehicles.Vehicle) | Main component holding vehicle data. |
| [VehicleWheelEvent](xref:Unity.Vehicles.VehicleWheelEvent) | Dynamic Buffer component for vehicle wheel events (wheel added, wheel removed, etc...). |
| [VehicleUpdateWheelData](xref:Unity.Vehicles.VehicleUpdateWheelData) | Enableable component triggering an update of vehicle wheel datas from the wheel entities. |
| [VehicleUpdateColliders](xref:Unity.Vehicles.VehicleUpdateColliders) | Enableable component triggering an update of the vehicle compound collider (mainly used internally for wheel add/remove). |
| [VehicleControl](xref:Unity.Vehicles.VehicleControl) | Main component holding vehicle control data: transmission, steering, acceleration, etc... |
| [EngineStartStop](xref:Unity.Vehicles.EngineStartStop) | Enableable component triggering engine start/stop. |
| [VehicleControlEvent](xref:Unity.Vehicles.VehicleControlEvent) | Dynamic Buffer component holding vehicle control events (engine start/stop, shift up, shift down, etc...). |
| [Wheel](xref:Unity.Vehicles.Wheel) | Main component holding wheel data. |
| [WheelOnVehicleReference](xref:Unity.Vehicles.WheelOnVehicleReference) | Component storing data about a wheel's index in the wheels buffer of the vehicle it's attached to. |
| [WheelControl](xref:Unity.Vehicles.WheelControl) | Main component holding wheel control data: max angles, torque/brake coefficients, etc... |

| **Systems** | **Description**|
| :--- | :--- |
| [VehiclePhysicsSystem](xref:Unity.Vehicles.VehiclePhysicsSystem) | System responsible for solving vehicle and wheel physics. |
| [VehicleControlSystem](xref:Unity.Vehicles.VehicleControlSystem) | System responsible for controlling vehicles given some player inputs. |
| [VehicleCollidersSystem](xref:Unity.Vehicles.VehicleCollidersSystem) | System responsible for updating the vehicle compound collider in order to include wheel protector colliders. |
| [VehicleContactModificationSystem](xref:Unity.Vehicles.VehicleContactModificationSystem) | System responsible for modifying vehicle contacts in order to smooth out certain types of collisions. |
| [VehicleWheelEventsClearSystem](xref:Unity.Vehicles.VehicleWheelEventsClearSystem) | System responsible for clearing vehicle events. |

| **Utilities** | **Description**|
| :--- | :--- |
| [VehicleUtilities](xref:Unity.Vehicles.VehicleUtilities) | Contains various utility functions related to vehicles and wheels. |
