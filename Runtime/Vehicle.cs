using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using UnityEngine.Serialization;

namespace Unity.Vehicles
{
    /// <summary>
    /// Contains core vehicle data and settings needed for vehicle simulation.
    /// For vehicle controls check <see cref="VehicleControl"/>.
    /// </summary>
    public struct Vehicle : IComponentData
    {
        /// <summary>
        /// Vehicle physics collider.
        /// </summary>
        public BlobAssetReference<Collider> BodyCollider;

        /// <summary>
        /// Number of physics sub-steps.
        /// These are additional physics steps that the vehicle will use within
        /// one global physics step.
        /// </summary>
        public int SubstepCount;

        /// <summary>
        /// Dimensions of the vehicle, expressed in m.
        /// </summary>
        public float3 Dimensions;

        /// <summary>
        /// Contact filtering
        /// </summary>
        public bool UseContactFiltering;

        /// <summary>
        /// Cosine of angle threshold for collision smoothing between the vehicle and mesh colliders.
        ///
        /// If, at a given contact point between the vehicle and a mesh collider, the angle between the contact normal
        /// and the mesh surface normal is larger than this threshold and does not exceed 90 degrees minus this threshold,
        /// the normal of the corresponding contact will be set to the surface normal, ensuring smooth collisions with the
        /// terrain surface.
        /// </summary>
        public float ContactSurfaceNormalAngleThresholdCos;

        /// <summary>
        /// Cosine of angle threshold for detecting hard terrain features.
        ///
        /// Corresponds to a threshold on the delta angle between the inverted vehicle's velocity at a given contact
        /// point and the colliding terrain feature's surface normal.
        /// Any surface feature leading to a delta angle that is larger than the threshold will be considered a hard terrain
        /// feature and contacts with this feature will not be filtered, ensuring collisions with hard terrain features
        /// are retained, enabling accurate collision response.
        /// </summary>
        public float HardTerrainFeatureAngleThresholdCos;

        /// <summary>
        /// Reference for internal speed value comparisons that depend on the vehicle size.
        /// Calculated based on the vehicle dimensions.
        /// </summary>
        internal float VirtualScale;

        /// <summary>
        /// Is the anti-creep position set?
        /// When true, the wheel will attempt to hold a fixed position to prevent
        /// creep on slopes.
        /// </summary>
        internal bool AntiCreepReferenceIsSet;

        /// <summary>
        /// Position of the anti-creep reference. The wheel will attempt
        /// to hold this position when stationary.
        /// </summary>
        internal float3 AntiCreepReferencePosition;
    }

    /// <summary>
    /// Stores events related to the vehicle wheels
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct VehicleWheelEvent : IBufferElementData
    {
        /// <summary>
        /// The event type
        /// </summary>
        public enum VehicleWheelEventType
        {
            /// <summary>
            /// When a wheel was added
            /// </summary>
            WheelAdded,
            /// <summary>
            /// When a wheel was removed
            /// </summary>
            WheelRemoved,
        }

        /// <summary>
        /// The event type
        /// </summary>
        public VehicleWheelEventType wheelEventType;
        /// <summary>
        /// The wheel entity affected by the event
        /// </summary>
        public Entity WheelEntity;
        /// <summary>
        /// The wheel suspension's local transform relative to the vehicle, at the moment of the event
        /// </summary>
        public RigidTransform WheelSuspensionLocalTransform;
        /// <summary>
        /// The wheel angular velocity, at the moment of the event
        /// </summary>
        public float WheelAngularVelocity;
    }

    /// <summary>
    /// Enableable component triggering an update of the vehicle wheel datas
    /// </summary>
    public struct VehicleUpdateWheelData : IComponentData, IEnableableComponent
    {
    }

    /// <summary>
    /// Enableable component triggering an update of the vehicle compound collider
    /// </summary>
    public struct VehicleUpdateColliders : IComponentData, IEnableableComponent
    {
    }

    /// <summary>
    /// Cleanup component used by vehicle systems to handle vehicle destruction
    /// </summary>
    public struct VehicleColliderCleanup : ICleanupComponentData
    {
    }
}