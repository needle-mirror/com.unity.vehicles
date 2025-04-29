using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Vehicles;
using UnityEngine;
using UnityEngine.Serialization;
using static Unity.Vehicles.VehicleControlAuthoring;

namespace Unity.Vehicles
{
    /// <summary>
    /// Additional functionality for the <see cref="Vehicle"/>.
    /// Powertrain, drivetrain, steering and in general everything related to controling the
    /// vehicle movement and behavior.
    /// </summary>
    [Serializable]
    public struct VehicleControl : IComponentData
    {
        /// <summary>
        /// Steering input without any processing.
        /// </summary>
        public float RawSteeringInput;

        /// <summary>
        /// Throttle input without any processing.
        /// </summary>
        public float RawThrottleInput;

        /// <summary>
        /// Brake input without any processing.
        /// </summary>
        public float RawBrakeInput;

        /// <summary>
        /// Handbrake input without any processing.
        /// </summary>
        public float HandbrakeInput;

        /// <summary>
        /// Transmission shift up input.
        /// </summary>
        public bool ShiftUpInput;

        /// <summary>
        /// Transmission shift down input.
        /// </summary>
        public bool ShiftDownInput;

        /// <summary>
        /// Engine start / stop input.
        /// </summary>
        public bool EngineStartStopInput;
    }

    /// <summary>
    /// Additional functionality for the <see cref="Vehicle"/>.
    /// Powertrain, drivetrain, steering and in general everything related to controling the
    /// vehicle movement and behavior.
    /// </summary>
    [Serializable]
    public struct VehicleControlData : IComponentData
    {
        // Engine

        /// <summary>
        /// Maximum engine angular velocity at which it can generate torque.
        /// Expressed in rad/s.
        /// </summary>
        public float EngineMaxAngularVelocity;

        /// <summary>
        /// Engine angular velocity at idle.
        /// Expressed in rad/s.
        /// </summary>
        public float EngineIdleAngularVelocity;

        /// <summary>
        /// Determines how the engine angular velocity is calculated from the wheel angular velocities.
        /// Maximum is recommended for vehicles with different wheel sizes.
        /// Minimum - only the slowest spinning driven wheel is taken into the account.
        /// Average - average of all the driven wheels.
        /// Maximum - only the fastest spinning driven wheel is taken into the account.
        /// </summary>
        public EngineAngularVelocityCalculationType EngineAngularVelocityCalculation;

        /// <summary>
        /// Maximum torque the engine can produce.
        /// The final torque value also depends on the EngineTorqueCurve.
        /// </summary>
        public float EngineMaxTorque;

        /// <summary>
        /// Normalized torque curve representing engine angular velocity on the X axis
        /// and the torque on the Y axis.
        /// </summary>
        public BlobAssetReference<SampledCurve> EngineTorqueCurve;

        /// <summary>
        /// The amount of negative torque created by the engine due to the losses,
        /// expressed as a percentage of the maximum engine torque.
        /// Engine braking is only applied when there is not throttle input.
        /// </summary>
        public float EngineBrakingIntensity;

        /// <summary>
        /// Angular inertia of the engine.
        /// Higher value will result in an engine that takes longer time to spin up/down.
        /// </summary>
        public float EngineAngularInertia;

        /// <summary>
        /// Is the engine running?
        /// </summary>
        public bool EngineIsRunning;

        /// <summary>
        /// Should the engine be automatically started on throttle input?
        /// </summary>
        public bool EngineStartOnThrottle;

        /// <summary>
        /// Engine angular velocity in rad/s.
        /// </summary>
        public float EngineAngularVelocity;

        /// <summary>
        /// Engine torque in N/m.
        /// </summary>
        public float EngineTorque;

        // Transmission

        /// <summary>
        /// Is the transmission automatic?
        /// Automatic transmission requires no user input for changing gears.
        /// </summary>
        public bool TransmissionIsAutomatic;

        /// <summary>
        /// List of the gear ratios for forward gears.
        /// The list starts with the 1st gear (D1).
        /// The values should be in a descending order
        /// since the higher the gear, the lower the gear ratio.
        /// </summary>
        public FixedList128Bytes<float> TransmissionForwardGearRatios;

        /// <summary>
        /// List of the gear ratios for the reverse gears.
        /// The list starts with the 1st reverse gear (R1).
        /// The values should be in a descending order.
        /// </summary>
        public FixedList128Bytes<float> TransmissionReverseGearRatios;

        /// <summary>
        /// A multiplier by which the forward and reverse gear ratios are multiplied.
        /// Commonly also called differential (diff) gear ratio.
        /// </summary>
        public float TransmissionFinalGearRatio;

        /// <summary>
        /// The angular velocity at which the automatic transmission will shift into a higher gear.
        /// </summary>
        public float TransmissionShiftUpAngularVelocity;

        /// <summary>
        /// The angular velocity at which the automatic transmission will shift into a lower gear.
        /// </summary>
        public float TransmissionShiftDownAngularVelocity;

        /// <summary>
        /// The gear the transmission is currently in.
        /// </summary>
        public int TransmissionCurrentGear;

        /// <summary>
        /// Current total gear ratio.
        /// Equals to the gear ratio of the currently selected gear
        /// multiplied the final gear ratio.
        /// </summary>
        public float TransmissionCurrentGearRatio;

        // Control

        /// <summary>
        /// Should the throttle and brake input be swapped when going in reverse?
        /// Usually true with the automatic transmission.
        /// </summary>
        public bool SwapThrottleAndBrakeInReverse;

        /// <summary>
        /// The speed of change of the steering angle.
        /// </summary>
        public float SteerSpeed;

        /// <summary>
        /// Maximum brake torque that can be applied to a wheel.
        /// Final wheel brake torque also depends on the brake coefficient for that wheel.
        /// </summary>
        public float MaxBrakeTorque;

        /// <summary>
        /// Wheel speed limiter, with braking applied once the set speed is exceeded.
        /// Set to 0 to deactivate.
        /// </summary>
        public float WheelAngularVelocityLimit;

        // Processed inputs

        /// <summary>
        /// Processed throttle input.
        /// This value might correspond to the RawThrottleInput or RawBrakeInput,
        /// depending on the SwapThrottleAndBrakeInReverse value and the direction of the
        /// travel of the vehicle.
        /// </summary>
        public float ThrottlePosition;

        /// <summary>
        /// Processed brake input.
        /// This value might correspond to the RawThrottleInput or RawBrakeInput,
        /// depending on the SwapThrottleAndBrakeInReverse value and the direction of the
        /// travel of the vehicle.
        /// </summary>
        public float BrakePosition;

        /// <summary>
        /// Processed steering input, with smoothing applied.
        /// </summary>
        public float SteeringPosition;

        /// <summary>
        /// The percentage of steering input at low speeds.
        /// </summary>
        public float SteeringSensitivityLowSpeed;

        /// <summary>
        /// The percentage of steering input at high speeds (<see cref="SteeringSensitivitySpeedReference"/>).
        /// </summary>
        public float SteeringSensitivityHighSpeed;

        /// <summary>
        /// The speed at which the <see cref="SteeringSensitivityHighSpeed"/> is in full effect.
        /// The steering sensitivity is lerped between the <see cref="SteeringSensitivityLowSpeed"/>
        /// and <see cref="SteeringSensitivityHighSpeed"/> from 0 to this value.
        /// </summary>
        public float SteeringSensitivitySpeedReference;

        // Aero Drag

        /// <summary>
        /// Coefficient of drag (Cd).
        /// </summary>
        public float AeroDragCoefficient;

        /// <summary>
        /// Aero force application point offset from the vehicle center of mass.
        /// </summary>
        public float3 AeroDragOffset;

        /// <summary>
        /// World position at which the aerodynamic srag is applied at.
        /// Determined by the vehicle center of mass and the drag offset value.
        /// </summary>
        public float3 AeroDragPosition;

        /// <summary>
        /// Total aerodynamic drag inpulse for the physics step.
        /// </summary>
        public float3 AeroDragImpulse;

        // Aero Downforce

        /// <summary>
        /// Aerodynamic downforce coefficient (Cl).
        /// </summary>
        public float AeroDownforceCoefficient;

        /// <summary>
        /// Aero downforce application point offset from the vehicle center of mass.
        /// </summary>
        public float3 AeroDownforceOffset;

        /// <summary>
        /// Position at which the downforce is applied.
        /// </summary>
        public float3 AeroDownforcePosition;

        /// <summary>
        /// Total downforce impulse for the physics step.
        /// </summary>
        public float3 AeroDownforceImpulse;

        // Calculated values

        /// <summary>
        /// Speed relative to the surface the vehicle is on. Only surfaces with Rigidbody on them are taken into account.
        /// </summary>
        public float RelativeForwardSpeed;

        /// <summary>
        /// Speed of the vehicle calculated from the wheel angular velocity and radius.
        /// Which wheel or wheels are taken into account when calculating this speed depends on the <see cref="EngineAngularVelocityCalculation"/>.
        /// </summary>
        public float WheelSpeed;
    }

    /// <summary>
    /// When enabled, <see cref="VehicleEngineControlJob"/> will read the value of the Start
    /// field and start or stop the engine.
    /// </summary>
    public struct EngineStartStop : IComponentData, IEnableableComponent
    {
        /// <summary>
        /// Starts the engine when set to true, stops the engine when set to false.
        /// </summary>
        public bool Start;
    }

    /// <summary>
    /// Events related to the <see cref="VehicleControl"/>.
    /// These can be used to signal to the vehicle that an action is needed.
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct VehicleControlEvent : IBufferElementData
    {
        /// <summary>
        /// The event type
        /// </summary>
        public enum Type
        {
            /// <summary>
            /// Starts the engine.
            /// </summary>
            StartEngine,

            /// <summary>
            /// Stops the engine.
            /// </summary>
            StopEngine,

            /// <summary>
            /// Shifts the transmission into a higher gear.
            /// </summary>
            ShiftUp,

            /// <summary>
            /// Shifts the transmission into a lower gear.
            /// </summary>
            ShiftDown,
        }

        /// <summary>
        /// The event type
        /// </summary>
        public Type EventType;

        /// <summary>
        /// Constructor for <see cref="VehicleControlEvent"/>.
        /// </summary>
        /// <param name="type">Type of the event.</param>
        public VehicleControlEvent(Type type)
        {
            EventType = type;
        }
    }
}