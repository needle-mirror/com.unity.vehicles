# Samples

## Installation

The samples can be installed via the *Samples* tab of the Vehicles Package Manager window (*Window > Package Manager > Vehicles > Samples*).

## Minimal Vehicle Sample

This sample demonstrates a minimal vehicle setup. The scene includes a "MinimalVehicle" prefab, which is controlled by a "VehicleController" object.


## Advanced Vehicle Sample

This sample includes several vehicle types and a more complex enviromnent.

### Vehicles Prefabs
The sample package contains the following vehicle prefabs:
* **Car** - a sports car with 4WD setup and stiff suspension.
* **Pickup** - a typical pickup truck with RWD and soft suspension.
* **MonsterTruck** - similar to **Pickup**, but with long suspension travel, large wheels and a more powerful engine.
* **Truck** - a heavy semi truck with a large number of forward and reverse gears. Similar setup can be used for other heavy vehicles such as busses.
* **Trailer** - a simple trailer setup that passively attaches to the **Truck** through a joint, without an ability to be controlled.

### Game Prefabs
* **GameSetup** - handles singleplayer game setup, including spawning the vehicles.
* **PlayerController** - handles player control of the vehicle, including input.
* **VehicleCamera** - 3rd person camera prefab. Each vehicle has a *CameraTarget* `GameObject` which can used to adjust the camera tracking position.

### Controls
Default keyboard bindings are:
* **W/A/S/D** - direction controls
* **Space** - handbrake
* **E** - engine start/stop
* **L. Shift** - shift up
* **L. Ctrl** - shift down
* **1, 2, 3, 4, 5 number keys** - select a vehicle
