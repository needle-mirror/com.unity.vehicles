using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.Vehicles
{
    /// <summary>
    /// Authoring component for vehicle control
    /// </summary>
    [RequireComponent(typeof(VehicleAuthoring))]
    [DisallowMultipleComponent]
    public class VehicleControlAuthoring : MonoBehaviour
    {
        private const float RPM_TO_ANGULAR = 0.10472f;

        /// <summary>
        /// The type of engine angular velocity calculation
        /// </summary>
        public enum EngineAngularVelocityCalculationType
        {
            /// <summary>
            /// Minimum
            /// </summary>
            Minimum,
            /// <summary>
            /// Average
            /// </summary>
            Average,
            /// <summary>
            /// Maximum
            /// </summary>
            Maximum
        }

        /// <summary>
        /// Maximum engine RPM at which it can generate torque.
        /// </summary>
        [Header("Engine")]
        [Tooltip("Maximum engine RPM at which it can generate torque.")]
        public float EngineMaxRPM = 7000f;

        /// <summary>
        /// Engine RPM at idle.
        /// </summary>
        [Tooltip("Engine RPM at idle.")]
        public float EngineIdleRPM = 700f;

        /// <summary>
        /// Determines how the engine angular velocity is calculated from the wheel angular velocities.
        /// Maximum is recommended for vehicles with different wheel sizes.
        /// Minimum - only the slowest spinning driven wheel is taken into the account.
        /// Average - average of all the driven wheels.
        /// Maximum - only the fastest spinning driven wheel is taken into the account.
        /// </summary>
        [Tooltip("Determines how the engine angular velocity is calculated from the wheel angular velocities. " +
            "Maximum is recommended for vehicles with different wheel sizes. " +
            "Minimum - only the slowest spinning driven wheel is taken into the account. " +
            "Average - average of all the driven wheels. " +
            "Maximum - only the fastest spinning driven wheel is taken into the account.")]
        public EngineAngularVelocityCalculationType EngineAngularVelocityCalculation = EngineAngularVelocityCalculationType.Minimum;

        /// <summary>
        /// Maximum torque the engine can produce.
        /// The final torque value also depends on the EngineTorqueCurve.
        /// </summary>
        [Tooltip("Maximum torque the engine can produce. The final torque value also depends on the EngineTorqueCurve.")]
        public float EngineMaxTorque = 300f;

        /// <summary>
        /// Normalized torque curve representing engine angular velocity on the X axis
        /// and the torque on the Y axis.
        /// </summary>
        [Tooltip("Normalized torque curve representing engine angular velocity on the X axis and the torque on the Y axis. ")]
        public SampledAnimationCurve EngineTorqueCurve;

        /// <summary>
        /// The amount of negative torque created by the engine due to the losses,
        /// expressed as a percentage of the maximum engine torque.
        /// Engine braking is only applied when there is not throttle input.
        /// </summary>
        [Tooltip("The amount of negative torque created by the engine due to the losses, " +
            "expressed as a percentage of the maximum engine torque. Engine braking is only applied when there is not throttle input.")]
        public float EngineBrakingIntensity = 0.25f;

        /// <summary>
        /// Angular inertia of the engine.
        /// Higher value will result in an engine that takes longer time to spin up/down.
        /// </summary>
        [Tooltip("Angular inertia of the engine. Higher value will result in an engine that takes longer time to spin up/down.")]
        public float EngineAngularInertia = 0.5f;

        /// <summary>
        /// Should the engine be automatically started on vehicle awake?
        /// </summary>
        [Tooltip("Should the engine be automatically started on vehicle awake?")]
        public bool EngineStartOnAwake = true;

        /// <summary>
        /// Should the engine be automatically started on throttle input?
        /// </summary>
        [Tooltip("Should the engine be automatically started on throttle input?")]
        public bool EngineStartOnThrottle = true;

        /// <summary>
        /// Is the transmission automatic?
        /// Automatic transmission requires no user input for changing gears.
        /// </summary>
        [Header("Transmission")]
        [Tooltip("Is the transmission automatic? Automatic transmission requires no user input for changing gears.")]
        public bool TransmissionIsAutomatic = true;

        /// <summary>
        /// List of the gear ratios for forward gears.
        /// The list starts with the 1st gear (D1).
        /// The values should be in a descending order
        /// since the higher the gear, the lower the gear ratio.
        /// </summary>
        [Tooltip("List of the gear ratios for forward gears.  The list starts with the 1st gear (D1). " +
            "The values should be in a descending order since the higher the gear, the lower the gear ratio.")]
        public List<float> TransmissionForwardGearRatios = new List<float> { 8f, 5.5f, 4f, 3f, 2.2f, 1.7f };

        /// <summary>
        /// List of the gear ratios for the reverse gears.
        /// The list starts with the 1st reverse gear (R1).
        /// The values should be in a descending order.
        /// </summary>
        [Tooltip("List of the gear ratios for the reverse gears. The list starts with the 1st reverse gear (R1). " +
            "The values should be in a descending order.")]
        public List<float> TransmissionReverseGearRatios = new List<float> { -7f };

        /// <summary>
        /// A multiplier by which the forward/reverse gear ratios are multiplied.
        /// Commonly also called differential (diff) gear ratio.
        /// </summary>
        [Tooltip("A multiplier by which the forward/reverse gear ratios are multiplied. " +
            "Commonly also called differential (diff) gear ratio.")]
        public float TransmissionFinalGearRatio = 3f;

        /// <summary>
        /// The RPM at which the automatic transmission will shift into a higher gear.
        /// </summary>
        [Tooltip("The RPM at which the automatic transmission will shift into a higher gear.")]
        public float TransmissionShiftUpRPM = 6000f;

        /// <summary>
        /// The RPM at which the automatic transmission will shift into a lower gear.
        /// </summary>
        [Tooltip("The RPM at which the automatic transmission will shift into a lower gear.")]
        public float TransmissionShiftDownRPM = 2000f;

        /// <summary>
        /// Should the throttle and brake input be swapped when going in reverse?
        /// Usually true with the automatic transmission.
        /// </summary>
        [Header("Control")]
        [Tooltip("Should the throttle and brake input be swapped when going in reverse? " +
            "Usually true with the automatic transmission.")]
        public bool SwapThrottleAndBrakeInReverse = true;

        /// <summary>
        /// The speed of change of the steering angle.
        /// </summary>
        [Tooltip("The speed of change of the steering angle.")]
        public float SteerSpeed = 5f;

        /// <summary>
        /// The percentage of steering input at low speeds.
        /// </summary>
        [Tooltip("The percentage of steering input at low speeds.")]
        public float SteeringSensitivityLowSpeed = 1f;

        /// <summary>
        /// The percentage of steering input at high speeds (<see cref="SteeringSensitivitySpeedReference"/>).
        /// </summary>
        [Tooltip("The percentage of steering input at high speeds.")]
        public float SteeringSensitivityHighSpeed = 0.2f;

        /// <summary>
        /// The speed at which the <see cref="SteeringSensitivityHighSpeed"/> is in full effect.
        /// The steering sensitivity is lerped between the <see cref="SteeringSensitivityLowSpeed"/>
        /// and <see cref="SteeringSensitivityHighSpeed"/> from 0 to this value.
        /// </summary>
        [Tooltip("The speed at which the SteeringSensitivityHighSpeed is in full effect." +
            "The steering sensitivity is lerped between the SteeringSensitivityLowSpeed" +
            "and SteeringSensitivityHighSpeed from 0 to this value.")]
        public float SteeringSensitivitySpeedReference = 100f;

        /// <summary>
        /// Maximum brake torque that can be applied to a wheel.
        /// Final wheel brake torque also depends on the brake coefficient for that wheel.
        /// </summary>
        [Tooltip("Maximum brake torque that can be applied to a wheel. " +
            "Final wheel brake torque also depends on the brake coefficient for that wheel.")]
        public float MaxBrakeTorque = 40000;

        /// <summary>
        /// Wheel speed limiter, with braking applied once the set speed is exceeded.
        /// Set to lesser or equal to 0 to deactivate.
        /// </summary>
        [Tooltip("Wheel speed limiter, with braking applied once the set speed is exceeded. " +
                 "Set to lesser or equal to 0 to deactivate.")]
        public float WheelAngularVelocityLimit = -1f;

        /// <summary>
        /// Aerodyamic drag coefficient (Cd).
        /// </summary>
        [Header("Aerodynamics")]
        [Tooltip("Aerodyamic drag coefficient (Cd).")]
        public float AeroDragCoefficient = 0.35f;

        /// <summary>
        /// Aero force application point offset from the vehicle center of mass.
        /// </summary>
        [Tooltip("Aero force application point offset from the vehicle center of mass.")]
        public float3 AeroDragOffset;

        /// <summary>
        /// Aerodynamic downforce coefficient (Cl).
        /// </summary>
        [Tooltip("Aerodynamic downforce coefficient (Cl).")]
        public float AeroDownforceCoefficient = 0.1f;

        /// <summary>
        /// Aero downforce application point offset from the vehicle center of mass.
        /// </summary>
        [Tooltip("Aero downforce application point offset from the vehicle center of mass.")]
        public float3 AeroDownforceOffset;


        private class Baker : Baker<VehicleControlAuthoring>
        {
            public override void Bake(VehicleControlAuthoring authoring)
            {
                VehicleAuthoring vehicleAuthoring =
                    authoring.GetComponent<VehicleAuthoring>();

                if (vehicleAuthoring == null)
                {
                    UnityEngine.Debug.LogError($"ERROR: VehicleControl needs to be on the same GameObject as the vehicle controller authoring");
                    return;
                }

                Entity entity = GetEntity(authoring, TransformUsageFlags.Dynamic);

                VehicleControlData vehicleControlData = new VehicleControlData
                {
                    // Engine
                    EngineMaxAngularVelocity =
                        math.clamp(authoring.EngineMaxRPM * RPM_TO_ANGULAR, math.EPSILON, math.INFINITY),
                    EngineIdleAngularVelocity =
                        math.clamp(authoring.EngineIdleRPM * RPM_TO_ANGULAR, math.EPSILON, math.INFINITY),
                    EngineAngularVelocityCalculation = authoring.EngineAngularVelocityCalculation,
                    EngineMaxTorque = math.clamp(authoring.EngineMaxTorque, 0f, math.INFINITY),
                    EngineTorqueCurve = authoring.EngineTorqueCurve.CreateBlob(this),
                    EngineBrakingIntensity = math.saturate(authoring.EngineBrakingIntensity),
                    EngineAngularInertia =
                        math.clamp(authoring.EngineAngularInertia, math.EPSILON, math.INFINITY),
                    EngineStartOnThrottle = authoring.EngineStartOnThrottle,

                    // Transmission
                    TransmissionIsAutomatic = authoring.TransmissionIsAutomatic,
                    TransmissionForwardGearRatios = ConvertListToFixedList(in authoring.TransmissionForwardGearRatios),
                    TransmissionReverseGearRatios = ConvertListToFixedList(in authoring.TransmissionReverseGearRatios),
                    // Not using infinity as an upper limit as the torque would be infinite in that case,
                    // causing a NaN on the Rigidbody
                    TransmissionFinalGearRatio = math.clamp(authoring.TransmissionFinalGearRatio, math.EPSILON, 10000f),
                    TransmissionShiftUpAngularVelocity =
                        math.clamp(authoring.TransmissionShiftUpRPM * RPM_TO_ANGULAR, 0, math.INFINITY),
                    TransmissionShiftDownAngularVelocity =
                        math.clamp(authoring.TransmissionShiftDownRPM * RPM_TO_ANGULAR, 0, math.INFINITY),

                    // Control
                    SwapThrottleAndBrakeInReverse = authoring.SwapThrottleAndBrakeInReverse,
                    SteerSpeed = math.clamp(authoring.SteerSpeed, 0, math.INFINITY),
                    SteeringSensitivityLowSpeed = math.clamp(authoring.SteeringSensitivityLowSpeed, 0, 1),
                    SteeringSensitivityHighSpeed = math.clamp(authoring.SteeringSensitivityHighSpeed, 0, 1),
                    SteeringSensitivitySpeedReference = math.clamp(authoring.SteeringSensitivitySpeedReference, 0, math.INFINITY),
                    MaxBrakeTorque = math.clamp(authoring.MaxBrakeTorque, 0, math.INFINITY),
                    WheelAngularVelocityLimit = math.clamp(authoring.WheelAngularVelocityLimit, 0, math.INFINITY),

                    // Aero
                    AeroDragCoefficient = authoring.AeroDragCoefficient,
                    AeroDragOffset = authoring.AeroDragOffset,
                    AeroDownforceCoefficient = authoring.AeroDownforceCoefficient,
                    AeroDownforceOffset = authoring.AeroDownforceOffset,
                };

                // Check that the shift angular velocities are valid
                if (vehicleControlData.TransmissionShiftUpAngularVelocity >= vehicleControlData.EngineMaxAngularVelocity)
                {
                    UnityEngine.Debug.LogWarning("Transmission ShiftUpAngularVelocity should be lower than the EngineMaxAngularVelocity!");
                    vehicleControlData.TransmissionShiftUpAngularVelocity = vehicleControlData.EngineMaxAngularVelocity - math.EPSILON;
                }

                if (vehicleControlData.TransmissionShiftUpAngularVelocity <= vehicleControlData.TransmissionShiftDownAngularVelocity)
                {
                    UnityEngine.Debug.LogWarning("Transmission ShiftUpRPM should be higher than ShiftDownRPM!");
                    vehicleControlData.TransmissionShiftUpAngularVelocity = vehicleControlData.TransmissionShiftDownAngularVelocity + math.EPSILON;
                }

                AddComponent(entity, vehicleControlData);
                AddComponent(entity, new VehicleControl());

                AddBuffer<VehicleControlEvent>(entity);

                AddComponent(entity, new EngineStartStop
                {
                    Start = authoring.EngineStartOnAwake,
                });
                SetComponentEnabled<EngineStartStop>(entity, authoring.EngineStartOnAwake);
            }
        }

        private void Reset()
        {
            EngineTorqueCurve.AnimationCurve = new AnimationCurve()
            {
                keys = new Keyframe[]
                {
                    new Keyframe(0f, 0.5f),
                    new Keyframe(1f, 1f)
                }
            };
            EngineTorqueCurve.SamplesCount = 3;
        }


        private static FixedList128Bytes<float> ConvertListToFixedList(in List<float> list)
        {
            FixedList128Bytes<float> fixedList = new FixedList128Bytes<float>();
            int size = math.min(list.Count, 32);
            for (int i = 0; i < size; i++)
            {
                fixedList.Add(list[i]);
            }
            return fixedList;
        }
    }
}