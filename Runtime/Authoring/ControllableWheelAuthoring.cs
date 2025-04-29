using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Vehicles
{
    /// <summary>
    /// Authoring component representing a controllable vehicle wheel. Works together with <see cref="VehicleControlAuthoring"/>
    /// </summary>
    [RequireComponent(typeof(WheelAuthoring))]
    [DisallowMultipleComponent]
    public class ControllableWheelAuthoring : MonoBehaviour
    {
        /// <summary>
        /// Maximum wheel steer angle, in degrees.
        /// Can be negative to flip the steer direction.
        /// </summary>
        [Range(-90f, 90f)]
        [UnityEngine.Tooltip("Maximum wheel steer angle, in degrees. Can be negative to flip the steer direction.")]
        public float MaxSteerAngleDegrees;

        /// <summary>
        /// This value represents the additional steering angle that will be applied
        /// to the wheel on the inside of the turning circle.
        /// There is no effect if the wheel does not share an axle with an another wheel.
        /// Set to 0 to disable.
        /// </summary>
        [Range(-1f, 1f)]
        [UnityEngine.Tooltip("This value represents the additional steering angle that will be " +
            "applied to the wheel on the inside of the turning circle. There is no effect if the wheel " +
            "does not share an axle with an another wheel. Set to 0 to disable.")]
        public float AckermanCoefficient = 0.7f;

        /// <summary>
        /// The amount of motor torque this wheel can receive as a percentage
        /// of the maximum available torque for this wheel.
        /// </summary>
        [Range(0f, 1f)]
        [UnityEngine.Tooltip("The amount of motor torque this wheel can receive as a percentage of " +
            "the maximum available torque for this wheel.")]
        public float MotorTorqueCoefficient = 1f;

        /// <summary>
        /// The amount of brake torque this wheel can receive as a percentage
        /// of the maximum brake torque for this wheel.
        /// </summary>
        [Range(0f, 1f)]
        [UnityEngine.Tooltip("The amount of brake torque this wheel can receive as a percentage of " +
            "the maximum brake torque for this wheel.")]
        public float BrakeTorqueCoefficient = 1f;

        /// <summary>
        /// Determines the strength of the handbrake for this wheel.
        /// </summary>
        [Range(0f, 1f)]
        [UnityEngine.Tooltip("Determines the strength of the handbrake for this wheel.")]
        public float HandbrakeCoefficient = 1f;

        /// <summary>
        /// The amount of spin resistance torque this wheel has when no motor torque is applied.
        /// </summary>
        public float SpinResistanceTorque;

        private class Baker : Baker<ControllableWheelAuthoring>
        {
            public override void Bake(ControllableWheelAuthoring authoring)
            {
                Entity entity = GetEntity(authoring, TransformUsageFlags.None);
                AddComponent(entity, new WheelControl
                {
                    MaxSteerAngle = math.radians(authoring.MaxSteerAngleDegrees),
                    AckermanCoefficient = math.clamp(authoring.AckermanCoefficient, -1f, 1f),
                    MotorTorqueCoefficient = math.saturate(authoring.MotorTorqueCoefficient),
                    BrakeTorqueCoefficient = math.saturate(authoring.BrakeTorqueCoefficient),
                    HandbrakeTorqueCoefficient = math.saturate(authoring.HandbrakeCoefficient),
                    SpinResistanceTorque = math.max(0f, authoring.SpinResistanceTorque),
                });
            }
        }
    }
}