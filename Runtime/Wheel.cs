using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Serialization;

namespace Unity.Vehicles
{
    /// <summary>
    /// Contains wheel settings.
    /// For dynamic wheel data refer to <see cref="WheelOnVehicle"/>.
    /// </summary>
    public struct Wheel : IComponentData
    {
        /// <summary>
        /// Entity that will be rotated in order to represent wheel spin.
        /// </summary>
        public Entity SpinEntity;

        /// <summary>
        /// Radius of the wheel, expressed in m.
        /// Not affected by the scale.
        /// </summary>
        public float Radius;

        /// <summary>
        /// Width of the wheel, expressed in m.
        /// </summary>
        public float Width;

        /// <summary>
        /// Mass of the wheel, expressed in kg.
        /// The mass of the wheel affects only its angular inertia, which determines how long the wheel
        /// takes to spin up.
        /// Unrealistically small values can cause the simulation to require more sub-steps or result in rotational
        /// jitter when the vehicle is stationary.
        /// </summary>
        public float Mass;

        /// <summary>
        /// Represents the wheel collider for this wheel.
        /// This collider will be used for the wheel collider casts necessary for vehicle physics. It may also be used
        /// for the wheel protector collider if the option is enabled.
        /// </summary>
        public BlobAssetReference<Physics.Collider> WheelCollider;

        /// <summary>
        /// Maximum suspension travel / extension, expressed in m.
        /// For best suspension behavior the recommended length is more than the travel
        /// expected per one physics update, or the vehicle might bottom out frequently - which is when
        /// the suspension runs out of travel and the hard collision through a physics collider takes over.
        /// It is usually better to have more travel, and to adjust the vehicle ride height by
        /// adusting the suspension position instead.
        /// </summary>
        public float SuspensionMaxLength;

        /// <summary>
        /// Spring rate expressed in N/mm.
        /// Ideally this value would be set so that at rest, on flat ground, the suspension
        /// is about 30% compressed under the vehicle's own weight.
        /// Too low value and the vehicle will have less available suspension travel.
        /// Too high value and the vehicle will have little suspension travel available in extension,
        /// resulting in a wheel that loses contact with the ground easily.
        /// </summary>
        public float SuspensionSpringRate;

        /// <summary>
        /// Combined bump and rebound setting for the suspension damper.
        /// Determines how much the suspension will resist changes in suspension length.
        /// Higher values will result in a suspension more resistant bouncing and bobbing,
        /// but a too high value will reduce the effectiveness of the suspension.
        /// </summary>
        public float SuspensionDampingRate;

        /// <summary>
        /// Sharpness of the wheel visual movement along the suspension. The higher the value,
        /// the less visual smoothing there will be.
        /// </summary>
        public float VisualSuspensionSharpness;

        /// <summary>
        /// Maximum force the anti-roll bar can exert on one side of the suspension, in an
        /// attempt to reduce the body roll.
        /// Correctly adjusting the center of mass position should be the first step to addressing the
        /// body roll.
        /// </summary>
        public float AntiRollBarStiffness;

        /// <summary>
        /// Peak value of the friction curve. X represents slip and Y represents the grip coefficient.
        /// Peak friction happens when there is just enough slip to load the tire, but not enough that it
        /// starts sliding.
        /// High friction curve stiffness (X lesser than 0.1 or high Y values) should be avoided in case of low physics update rates
        /// or when using low number of sub-steps on the <see cref="Vehicle"/> to prevent the friction
        /// from over-correcting and causing jitter.
        /// </summary>
        public float2 FrictionCurveMaximum;

        /// <summary>
        /// Minimum value of the friction curve, after the peak.
        /// X represents slip and Y represents the grip coefficient.
        /// This value is relevant when the wheel has enough slip to have already passed the friction
        /// peak and is now sliding around.
        /// The X value should always be higher than the X value of the <see cref="FrictionCurveMaximum"/>.
        /// </summary>
        public float2 FrictionCurveMinimum;

        /// <summary>
        /// Determines how much the longitudinal slip affects the lateral friction.
        /// Wheels with value of 1 will have no lateral friction when the wheel is locked or
        /// there is wheel spin.
        /// </summary>
        public float FrictionCircleStrength;

        /// <summary>
        /// Should the wheel protector collider for this wheel be generated?
        /// This collider represents the wheel at full suspension compression, and is used to handle
        /// suspension bottoming out and impacts from directions that are not handled by the suspension.
        /// </summary>
        public bool AddWheelProtectorColliders;
    }

    /// <summary>
    /// Stores reference to the vehicle this wheel belongs to.
    /// </summary>
    public struct WheelOnVehicleReference : IComponentData
    {
        /// <summary>
        /// Vehicle this wheel belongs to.
        /// </summary>
        public Entity VehicleEntity;

        /// <summary>
        /// Index of the wheel in the vehicle wheels buffer.
        /// </summary>
        public int Index;
    }
}