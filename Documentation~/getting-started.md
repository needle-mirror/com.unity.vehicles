# Getting Started

> Note: for a fast and easy way to get started with a basic vehicle, import the [Minimal Vehicle Sample](./samples.md#minimal-vehicle-sample) into your project from the *Samples* tab on the package's page in the *Package Manager*.


## Vehicle Setup

#### Vehicle
Here is how a vehicle object should be setup and authored:
* Create a new GameObject, and add a `VehicleAuthoring` component to it. We will refer to this as the "root vehicle object".
    * `VehicleAuthoring` contains general settings for the vehicle.
    * This will automatically add a `Rigidbody` component to the object.
    * This GameObject's forward, up, right axes must correspond to the vehicle's forward, up, right.
    * This GameObject's scale must be (1,1,1).
* Set the mass of the vehicle `Rigidbody` to an appropriate value. A reasonable default would be 1500.
    * The default vehicle wheel suspension parameters are configured with the assumption that the vehicle has a high mass. If the default mass of 1 is kept and the wheel suspension parameters aren't tweaked to match this mass, the vehicle risks flying off once its wheels hit the ground.
* Add vehicle meshes and colliders as child objects of this root vehicle object.
    * These child mesh and collider GameObjects can have any scale.
* In the `VehicleAuthoring` component, configure the vehicle's dimensions using the `Dimensions` and `Dimensions Debug Center` fields.
    * Dimensions are used to calculate certain value thresholds related to vehicle size, and are also used for simplified aerodynamics.

At this point, we have a valid vehicle without wheels. You may want a prefab of a vehicle without wheels if wheels should be added at runtime, but in most cases vehicles will come with built-in wheels.

#### Wheels
Here is how to add wheels to the vehicle object. For each wheel:
* Create a new empty child GameObject of the root vehicle object, and add a `WheelAuthoring` component to it. We will refer to this as the "root wheel object".
    * `WheelAuthoring` contains settings for the wheel, such as dimensions, suspension, friction, etc...
    * This object's transform down direction will represent the direction of the suspension travel.
* Add wheel meshes as child objects of the root wheel object
* In the `WheelAuthoring` component, assign any wheel mesh that should spin with the wheel as the `WheelSpinObject`.
    * If multiple meshes should spin with the wheel, put them all under the same root object and assign this root object as the `WheelSpinObject`.
    * Meshes that should be parented to the wheel but shouldn't spin with the wheel, such as fenders or visual parts of the suspension, can simply be left as children of the root wheel object.
* In order to tell the vehicle about the wheels it has, go back to the root vehicle object, and assign the wheels to the `Wheels` list in the `VehicleAuthoring` component. You may also use the "Auto Detect Wheels" button in the `VehicleAuthoring` component's inspector to automatically fill the `VehicleAuthoring > Wheels` list with any `WheelAuthoring` objects found in the hierarchy.
* If the wheels are arranged in a standard two-wheels-per-axle configuration, they can also be added to `VehicleAuthoring > Axle Paired Wheels`. This allows for features like the anti-roll bar and Ackerman steering to be applied if necessary.

At this point, we have a valid vehicle with wheels. However, the vehicle isn't controllable yet.

#### Making the vehicle controllable

This package comes with built-in `VehicleControlAuthoring` and `ControllableWheelAuthoring` components for handling vehicle control: engine, transmission, steering, braking, etc... They represent a basic implementation of a typical vehicle.

> These components are optional. You may choose to omit them completely from the vehicle prefab for objects that have wheels and suspension but no "control" (such as a truck trailer), or you may choose to write your own version of vehicle control components and systems.

Here is how to add vehicle control to the vehicle object:
* On the root vehicle object, add a `VehicleControlAuthoring` component.
    * This component handles vehicle engine, transmission, control, aerodynamics, etc...
* On each root wheel object, add a `ControllableWheelAuthoring` component
    * This component works together with `VehicleControlAuthoring`, and determines how each specific wheel is controlled: max steer angles, coefficient of engine torque, etc...
    * A typical vehicle would only have steering for its front wheels, so set the `Max Steer Angles Degrees` to 0 for rear wheels. You you give the front wheels a value of 20 as an example.

At this point, the vehicle would be ready to be controlled with inputs.

#### Example hierarchy

In summary, here is a simple example of a vehicle hierarchy:
* VehicleRoot *(has the `Rigidbody`, `VehicleAuthoring`, and optionally `VehicleControlAuthoring`)*
    * BodyMesh (has the vehicle body mesh renderer and collider(s))
    * WheelFL *(has `WheelAuthoring` and optionally `ControllableWheelAuthoring`)*
        * WheelMeshFL *(visual representation of the wheel)*
        * NonRotatingObject *(visual representation of the objects that follow the wheel position, such as fenders and bones, but not axis rotation)*
    * WheelFR
        * WheelMeshFR
        * NonRotatingObject
    * etc.

#### Vehicle Configuration

For more details on the various vehicle configuration components and parameters, see [Vehicle Configuration](./vehicle-configuration.md).


## Controlling the vehicle with inputs

The vehicle can be controlled by creating a system that writes to the various fields of the `VehicleControl` component on the vehicle Entity:
* `RawSteeringInput`
* `RawThrottleInput`
* `RawBrakeInput`
* `HandbrakeInput`
* `ShiftUpInput`
* `ShiftDownInput`
* `EngineStartStopInput`

> Note: this assumes the `VehicleControlAuthoring` and `ControllableWheelAuthoring` built-in components are used for vehicle control, and not a custom solution. A custom solution would directly write to the `WheelOnVehicle` dynamic buffer on the vehicle entity, and could use `VehicleControlSystem` as an inspiration.

See how the `MinimalVehicleControlSystem` in the "Minimal Vehicle Sample" transfers player inputs to the `VehicleControl` component, for an example:
```cs
[UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
[UpdateBefore(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(VehicleControlSystem))]
public partial struct MinimalVehicleControlSystem : ISystem
{
    public void OnUpdate(ref SystemState state)
    {
        MinimalInputActions.DefaultMapActions defaultMapActions = MinimalInputResources.InputActions.DefaultMap;

        foreach (var (controller, vehicleControl) in
                 SystemAPI.Query<MinimalPlayerController, RefRW<VehicleControl>>())
        {
            vehicleControl.ValueRW.RawSteeringInput = defaultMapActions.Steering.ReadValue<float>();
            vehicleControl.ValueRW.RawThrottleInput = defaultMapActions.Throttle.ReadValue<float>();
            vehicleControl.ValueRW.RawBrakeInput = defaultMapActions.Brake.ReadValue<float>();
            vehicleControl.ValueRW.HandbrakeInput = defaultMapActions.Handbrake.ReadValue<float>();
            vehicleControl.ValueRW.ShiftUpInput = default;
            if (defaultMapActions.ShiftUp.WasPressedThisFrame())
            {
                vehicleControl.ValueRW.ShiftUpInput = true;
            }

            vehicleControl.ValueRW.ShiftDownInput = default;
            if (defaultMapActions.ShiftDown.WasPressedThisFrame())
            {
                vehicleControl.ValueRW.ShiftDownInput = true;
            }

            vehicleControl.ValueRW.EngineStartStopInput = default;
            if (defaultMapActions.EngineStartStop.WasPressedThisFrame())
            {
                vehicleControl.ValueRW.EngineStartStopInput = true;
            }
        }
    }
}
```

The built-in `VehicleControlSystem` will then process the data in the `VehicleControl` and transfer it to wheels appropriately, based on all the settings in the `VehicleControlAuthoring` and `ControllableWheelAuthoring` components.


## Adding, removing, and modifying wheels at runtime

Wheels can be added to or removed from a vehicle at runtime.
* To remove a wheel at runtime, use `VehicleUtilities.TryRemoveWheel`.
* To add a wheel to a vehicle at runtime, use `VehicleUtilities.TryAddWheel`.

Wheel prefabs can be authored separately from a vehicle prefab, if needed. They could then be added to a wheel-less vehicle at runtime.

#### Changing wheel data at runtime

Wheel properties that do not affect the wheel collider (such as suspension and friction properties) can be updated at runtime by following these steps:
* Change wheel properties on the `Wheel` component of a vehicle's wheel entities.
* Enable the `VehicleUpdateWheelData` component on the vehicle entity.

Wheels are entities with a `Wheel` component, holding the general properties of the wheel (size, friction, suspension, etc...). However, when added to a Vehicle entity, the wheel data is transfered to a `WheelOnVehicle` dynamic buffer on the Vehicle entity, representing all of the vehicle's wheels along with some data that's only relevant to a wheel that is actually attached to a vehicle (suspension transform, suspension compression, etc....). For this reason, if you wish to change the wheel parameters in the `Wheel` component of the Wheel entity at runtime, you need to inform the vehicle entity of the fact that the data of one of its wheels has changed, so that it may update its `WheelOnVehicle` buffer accordingly. This is what the `VehicleUpdateWheelData` enableable component does.


## Reading vehicle/wheel data and events

Vehicles may require audio, visual effects, UI information, etc... that depend on vehicle data and events. Here is a breakdown of useful vehicle data for these cases:
* The `DynamicBuffer<VehicleControlEvent>` on the vehicle entity holds events for vehicle engine start/stop, and transmission shift up/down. It is cleared and then filled by the `VehicleControlSystem`.
* The `VehicleControlData` component on the vehicle entity holds a variety of data related to the vehicle engine, transmission, steering, brake, throttle, etc...
* The `DynamicBuffer<WheelOnVehicle>` on the vehicle entity holds a variety of data related to wheel suspension, friction/slip, velocity, etc...
* Wheel worldspace transforms can be computed from a `WheelOnVehicle` buffer element, by passing it as a parameter to the `VehicleUtilities.GetWheelWorldTransform()` utility function. A `VehicleUtilities.GetWheelTransforms()` is also available, for returning more wheel-related transforms such as the suspension start transform and direction.
* The `DynamicBuffer<VehicleWheelEvent>` on the vehicle entity holds wheel add/remove events. The events are cleared by the `VehicleWheelEventsClearSystem`, and are produced by the `VehicleUtilities.TryRemoveWheel` and `VehicleUtilities.TryAddWheel` utilities.


## Adding queryable colliders to wheels

In some cases, you might want to add colliders to wheels, so that you may query them using physics queries (raycasts). Here are the steps you should take to accomplish this:
* Add collider components to the wheel authoring objects. You can add colliders the root wheel object (the object that has the `WheelAuthoring` component) or any of its child objects.
* The root wheel object needs a `Rigidbody` component, if any colliders are added to the it or its children. An error message will be shown if a `Rigidbody` component is not added.
* If the wheel is part of a vehicle, the `Rigidbody` component on the root wheel object should be kinematic. If the wheel is being authored as an independent prefab, the wheel `Rigidbody` does not have to be kinematic.
    * This allows, for example, to have a wheel prefab that acts as a dynamic physics object when not attached to a vehicle. However once added to a vehicle, it can be made kinematic so that it will only serve as a way to raycast the wheel.
* The colliders added to the wheel should be set-up so that the vehicle's wheel collider casts won't detect them as hits. There are several ways to do this:
    1. The colliders on the wheel could be made into triggers.
    2. The colliders on the wheel could be given a layer that is part of the `WheelAuthoring`'s `Wheel Collider Layer Overrides > Exclude Layers`. This option can be more interesting if the raycasts that expect to detect wheel hits also expect to only hit non-trigger colliders.

This setup can either be done by hand, or done through code by using the `DynamicBuffer<VehicleWheelEvent>` events buffer on the vehicle entity, which prodices events whenever wheels are added or removed. These events are produced even for pre-baked vehicles that are instantiated (if the vehicle prefab already has 4 wheels, it will spawn with 4 "WheelAdded" events in its `DynamicBuffer<VehicleWheelEvent>` when instantiated). The reverse setup can be done when a wheel is removed with "WheelRemoved" events: make it collideable and make it a dynamic rigidbody, so that the wheel becomes a physics object.