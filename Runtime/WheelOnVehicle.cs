using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using UnityEngine.Serialization;

namespace Unity.Vehicles
{
    /// <summary>
    /// Holds dynamic/calcualted wheel data.
    /// For wheel settings refer to <see cref="Wheel"/>.
    /// </summary>
    [InternalBufferCapacity(4)]
    public struct WheelOnVehicle : IBufferElementData
    {
        /// <summary>
        /// Index of the wheel protector child collider in the vehicle compound collider.
        /// </summary>
        public int WheelProtectorChildColliderIndex;
        
        /// <summary>
        /// Local transform of the wheel suspension. This transform represents the 
        /// origin and direction of the suspension travel.
        /// </summary>
        public RigidTransform SuspensionLocalTransform;

        /// <summary>
        /// Wheel entity that shares the same axle as this wheel.
        /// </summary>
        public Entity AxlePairedWheelEntity;

        /// <summary>
        /// Index of the wheel entity that shares the same axle as this wheel.
        /// </summary>
        public int AxlePairedWheelIndex;

        // State that is typically set on wheel add or remove events
        
        /// <summary>
        /// The wheel entity
        /// </summary>
        public Entity Entity;

        /// <summary>
        /// Wheel that belongs to this VehicleWheel buffer.
        /// </summary>
        public Wheel Wheel;

        // State that is typically set by vehicle control systems

        /// <summary>
        /// Current motor torque of the wheel, expressed in Nm.
        /// </summary>
        public float MotorTorque;

        /// <summary>
        /// Current brake torque of the wheel, expressed in Nm.
        /// The value can only be positive, and any value >0 will 
        /// block motor torque from being applied.
        /// </summary>
        public float BrakeTorque;

        /// <summary>
        /// Maximum angular velocity the wheel can achieve.
        /// </summary>
        public float AngularVelocityLimit;

        // State that persists across frames and changes (netcode ghost state)

        /// <summary>
        /// Current steer angle of the wheel in degrees.
        /// </summary>
        public float SteerAngle;

        /// <summary>
        /// Current angular velocity of the wheel in radians per second.
        /// </summary>
        public float AngularVelocity;

        /// <summary>
        /// Rotation angle of the wheel, relative to the axle.
        /// The value is clamped to one full rotation and can be used to position the wheel visual.
        /// </summary>
        public float RotationAngle;

        /// <summary>
        /// Current length of the suspension, expressed in meters.
        /// </summary>
        public float SuspensionLength;
        
        /// <summary>
        /// Smoothed suspension length for wheel meshes, expressed in meters.
        /// </summary>
        public float VisualSuspensionLength;

        // Values that do not change during sub-stepping

        // =============================================================================================
        // State that is fully cleared and recalculated by the vehicle physics update

        // Hit

        /// <summary>
        /// Is the wheel touching the ground?
        /// </summary>
        public bool IsGrounded;

        /// <summary>
        /// Data from the collider cast.
        /// </summary>
        public ColliderCastHit WheelHit;

        // Suspension

        /// <summary>
        /// Total suspension impulse for the global physics step.
        /// </summary>
        public float3 SuspensionImpulse;

        // Friction 

        /// <summary>
        /// Total friction impulse for the global physics step.
        /// </summary>
        public float3 FrictionImpulse;

        /// <summary>
        /// Friction slip values. X represents the longitudinal slip, Y represents the lateral slip.
        /// Both values are normalized.
        /// Lateral slip value of 1 equals 90 degree lateral slip angle, while the 
        /// longitudinal slip value represents slip ratio of 1.
        /// </summary>
        public float2 FrictionSlip;

        /// <summary>
        /// Friction speed values. X represents the longitudinal speed, Y represents the lateral speed.
        /// Expressed in m/s.
        /// Both values are in world space. To get the effective speeds for moving platforms, objects, etc. 
        /// subtract the HitRigidbodyVelocity from this value.
        /// </summary>
        public float2 FrictionSpeed;

        // =============================================================================================

        // Static friction

        /// <summary>
        /// Is the anti-creep position set?
        /// When true, the wheel will attempt to hold a fixed position to prevent
        /// creep on slopes.
        /// </summary>
        public bool StaticFrictionReferenceIsSet;

        /// <summary>
        /// Position of the anti-creep reference. The wheel will attempt 
        /// to hold this position when stationary.
        /// </summary>
        public float3 StaticFrictionRefPosition;

        /// <summary>
        /// Static friction is disabled for one frame when true.
        /// Allows the vehicle to get out of the static friction lock.
        /// </summary>
        public bool DisableStaticFrictionSingleFrame;


        /// <summary>
        /// Stores <see cref="WheelOnVehicle"/> defaults.
        /// </summary>
        /// <returns><see cref="WheelOnVehicle"/> defaults.</returns>
        internal static WheelOnVehicle GetDefault()
        {
            return new WheelOnVehicle
            {
                AxlePairedWheelIndex = -1,
                WheelProtectorChildColliderIndex = -1,
            };
        }
    }

    internal struct TmpVehicleWheelData
    {
        public float InvMomentOfInertia;
        public float InvRadius;
        public float MomentOfInertia;
        
        public RigidTransform SteeredSuspensionWorldTransform;
        public RigidTransform WheelWorldTransform;

        public float3 WheelForward;
        public float3 WheelUp;
        public float3 WheelRight;
        public float3 SteeredSuspensionWorldDirection;
        public float3 CenteredThreadPoint;

        public float SubsteppedSuspensionLength;
        public float3 SuspensionStepImpulse;
        public float3 FrictionStepImpulse;
        public float3 HitRigidbodyVelocity;

        internal void UpdateSuspensionTransform(in RigidTransform vehicleTransform, in WheelOnVehicle wheelOnVehicle)
        {
            SteeredSuspensionWorldTransform = VehicleUtilities.GetSteeredSuspensionWorldTransform(in vehicleTransform, in wheelOnVehicle);
            SteeredSuspensionWorldDirection = VehicleUtilities.GetSuspensionWorldDirection(in vehicleTransform, in wheelOnVehicle);
            WheelForward = VehicleUtilities.GetWheelForward(in SteeredSuspensionWorldTransform);
            WheelRight = VehicleUtilities.GetWheelRight(in SteeredSuspensionWorldTransform);
            WheelUp = VehicleUtilities.GetWheelUp(in SteeredSuspensionWorldTransform);
            UpdateWheelTransforms(in wheelOnVehicle);
        }

        internal void UpdateWheelTransforms(in WheelOnVehicle wheelOnVehicle)
        {
            WheelWorldTransform = VehicleUtilities.GetWheelWorldTransform(in SteeredSuspensionWorldTransform,
                wheelOnVehicle.SuspensionLength);
            CenteredThreadPoint = VehicleUtilities.GetCenteredThreadPoint(in wheelOnVehicle, in SteeredSuspensionWorldTransform, 
                in WheelWorldTransform);
        }
    }
}