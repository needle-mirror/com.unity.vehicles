# Troubleshooting

Common issues that might be encountered:

### The vehicle is sliding around when stationary or jittering.
This can happen when the vehicle physics settings are too stiff for the given update rate.

Suggestions:
* Increase the number of the sub-steps on the `VehicleAuthoring`, and/or
* Increase the overall physics update rate, and/or
* Adjust the following:
    * `WheelAuthoring` `FrictionCurveMaximum` - too low X value or too high Y value can result in a friction that is too stiff.
    * `WheelAuthoring` `SuspensionDampingRate` - very high damper values might cause the suspension to over-correct itself.
    * `WheelAuthoring` `AntiRollBarStiffness` - extreme values can prevent the suspension from working correctly.
    * `Rigidbody` `CenterOfMass` - extreme values can cause jitter.
    * `Rigidbody` `InertiaTensor` and `Mass` - too low values can cause jitter.

### Vehicle is rolling or tipping over.
Suggestions:
* Check the `Rigidbody > Center Of Mass` is adjusted correctly (untick `Automatic Center Of Mass`). In general, the center of mass should be positioned in between the driver's seats, at the level of the seat cushion. Note that this is just a general guideline. Too high center of mass will result in the vehicle being prone of rolling or tipping over.

### Vehicle is not moving.
Suggestions:
* Verify that the `VehicleControlAuthoring` is set up.
* Check that there is a valid `AnimationCurve` assigned under `VehicleControlAuthoring` > `EngineTorqueCurve`.
* Check that the engine is started (`VehicleControlAuthoring.EngineStartOnAwake`, or `EngineStartStop` component).

### Vehicle is moving, but only slowly.
Suggestions:
* Check that there is adequate `EngineMaxTorque` for the vehicle mass.
* Check that the `TransmissionFinalGearRatio` and `TransmissionForwardGearRatios` / `TransmissionReverseGearRation` are high enough for the vehicle engine torque and mass. Very low values will result in little torque being passed on to the wheels.