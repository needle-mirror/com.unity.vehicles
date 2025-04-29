using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Serialization;
using Collider = Unity.Physics.Collider;
using Material = Unity.Physics.Material;

namespace Unity.Vehicles
{
    /// <summary>
    /// Layer overrides for wheel colliders
    /// </summary>
    [Serializable]
    public struct WheelColliderLayerOverrides
    {
        /// <summary>
        /// The layer override priority
        /// </summary>
        public int LayerOverridePriority;
        /// <summary>
        /// Included layers
        /// </summary>
        public LayerMask IncludeLayers;
        /// <summary>
        /// Excluded layers
        /// </summary>
        public LayerMask ExcludeLayers;
    }

    /// <summary>
    /// Authoring component for wheels
    /// </summary>
    [DisallowMultipleComponent]
    public class WheelAuthoring : MonoBehaviour
    {
        /// <summary>
        /// Data for the wheel friction curve
        /// </summary>
        [Serializable]
        public struct FrictionCurveData
        {
            /// <summary>
            /// Peak value of the friction curve. X represents slip and Y represents the grip coefficient.
            /// Peak friction happens when there is just enough slip to load the tire, but not enough that it
            /// starts sliding.
            /// High friction curve stiffness (X lesser than 0.1 or high Y values) should be avoided in case of low physics update rates
            /// or when using low number of sub-steps on the <see cref="Vehicle"/> to prevent the friction
            /// from over-correcting and causing jitter.
            /// </summary>
            [UnityEngine.Tooltip("Peak value of the friction curve. X represents slip and Y represents the grip coefficient. Peak friction happens when there is just enough slip to load the tire, but not enough that it starts sliding. High friction curve stiffness (X < 0.1 or high Y values) should be avoided in case of low physics update rates or when using low number of sub-steps on the Vehc to prevent the friction from over-correcting and causing jitter.")]
            public Vector2 FrictionCurveMaximum;

            /// <summary>
            /// Minimum value of the friction curve, after the peak.
            /// X represents slip and Y represents the grip coefficient.
            /// This value is relevant when the wheel has enough slip to have already passed the friction
            /// peak and is now sliding around.
            /// The X value should always be higher than the X value of the <see cref="FrictionCurveMaximum"/>.
            /// </summary>
            [UnityEngine.Tooltip("Minimum value of the friction curve, after the peak. X represents slip and Y represents the grip coefficient. This value is relevant when the wheel has enough slip to have already passed the friction peak and is now sliding around. The X value should always be higher than the X value of the FrictionCurveMaximum.")]
            public Vector2 FrictionCurveMinimum;

            /// <summary>
            /// Gets the default friction curve data
            /// </summary>
            /// <returns>The default friction curve data</returns>
            public static FrictionCurveData GetDefault()
            {
                return new FrictionCurveData
                {
                    FrictionCurveMaximum = new float2(0.2f, 1.75f),
                    FrictionCurveMinimum = new float2(0.6f, 1f),
                };
            }
        }

        const float SMALL_VALUE = 1e-5f;


        /// <summary>
        /// Object that will be rotated in order to represent wheel spin.
        /// </summary>
        [Header("General")]
        [UnityEngine.Tooltip("Object that will be rotated in order to represent wheel spin.")]
        public GameObject WheelSpinObject;

        /// <summary>
        /// Radius of the wheel, expressed in m.
        /// Not affected by the scale.
        /// </summary>
        [UnityEngine.Tooltip("Radius of the wheel, expressed in m. Not affected by the scale.")]
        public float Radius = 0.35f;

        /// <summary>
        /// Width of the wheel, expressed in m.
        /// </summary>
        [UnityEngine.Tooltip("Width of the wheel, expressed in m.")]
        public float Width = 0.2f;

        /// <summary>
        /// Mass of the wheel, expressed in kg.
        /// The mass of the wheel affects only its angular inertia, which determines how long the wheel
        /// takes to spin up.
        /// Unrealistically small values can cause the simulation to require more sub-steps or result in rotational
        /// jitter when the vehicle is stationary.
        /// </summary>
        [UnityEngine.Tooltip("Mass of the wheel, expressed in kg. The mass of the wheel affects only its angular inertia, which determines how long the wheel takes to spin up. Unrealistically small values can cause the simulation to require more sub-steps or result in rotational jitter when the vehicle is stationary.")]
        public float Mass = 30f;



        /// <summary>
        /// Maximum suspension travel / extension, expressed in m.
        /// For best suspension behavior the recommended length is more than the travel
        /// expected per one physics update, or the vehicle might bottom out frequently - which is when
        /// the suspension runs out of travel and the hard collision through a physics collider takes over.
        /// It is usually better to have more travel, and to adjust the vehicle ride height by
        /// adusting the suspension position instead.
        /// </summary>
        [Header("Suspension")]
        [UnityEngine.Tooltip("Maximum suspension travel / extension, expressed in m. For best suspension behavior the recommended length is more than the travel expected per one physics update, or the vehicle might bottom out frequently - which is when the suspension runs out of travel and the hard collision through a physics collider takes over. It is usually better to have more travel, and to adjust the vehicle ride height by adusting the suspension position instead.")]
        public float SuspensionMaxLength = 0.3f;

        /// <summary>
        /// Spring rate expressed in N/mm.
        /// Ideally this value would be set so that at rest, on flat ground, the suspension
        /// is about 30% compressed under the vehicle's own weight.
        /// Too low value and the vehicle will have less available suspension travel.
        /// Too high value and the vehicle will have little suspension travel available in extension,
        /// resulting in a wheel that loses contact with the ground easily.
        /// </summary>
        [UnityEngine.Tooltip("Spring rate expressed in N/mm. Ideally this value would be set so that at rest, on flat ground, the suspension is about 30% compressed under the vehicle's own weight. Too low value and the vehicle will have less available suspension travel. Too high value and the vehicle will have little suspension travel available in extension, resulting in a wheel that loses contact with the ground easily.")]
        public float SuspensionSpringRate = 50f;

        /// <summary>
        /// Combined bump and rebound setting for the suspension damper.
        /// Determines how much the suspension will resist changes in suspension length.
        /// Higher values will result in a suspension more resistant bouncing and bobbing,
        /// but a too high value will reduce the effectiveness of the suspension.
        /// </summary>
        [UnityEngine.Tooltip("Combined bump and rebound setting for the suspension damper. Determines how much the suspension will resist changes in suspension length. Higher values will result in a suspension more resistant bouncing and bobbing, but a too high value will reduce the effectiveness of the suspension.")]
        public float SuspensionDampingRate = 6000f;

        /// <summary>
        /// Sharpness of the wheel visual movement along the suspension. The higher the value,
        /// the less visual smoothing there will be.
        /// </summary>
        [UnityEngine.Tooltip("Sharpness of the wheel visual movement along the suspension. The higher the value, the less visual smoothing there will be.")]
        public float VisualSuspensionSharpness = 30f;

        /// <summary>
        /// Maximum force the anti-roll bar can exert on one side of the suspension, in an
        /// attempt to reduce the body roll.
        /// Correctly adjusting the center of mass position should be the first step to addressing the
        /// body roll.
        /// </summary>
        [UnityEngine.Tooltip("Maximum force the anti-roll bar can exert on one side of the suspension, in anattempt to reduce the body roll. Correctly adjusting the center of mass position should be the first step to addressing the body roll.")]
        public float AntiRollBarStiffness = 10000f;



        /// <summary>
        /// Friction curve.
        /// X represents slip and Y represents the grip coefficient.
        /// </summary>
        [Header("Friction")]
        [UnityEngine.Tooltip("Friction curve. X represents slip and Y represents the grip coefficient.")]
        public FrictionCurveData FrictionCurve = FrictionCurveData.GetDefault();

        /// <summary>
        /// Determines how much the longitudinal slip affects the lateral friction.
        /// Wheels with value of 1 will have no lateral friction when the wheel is locked or
        /// there is wheel spin.
        /// </summary>
        [Range(0, 1)]
        [UnityEngine.Tooltip("Determines how much the longitudinal slip affects the lateral friction. Wheels with value of 1 will have no lateral friction when the wheel is locked or there is wheel spin.")]
        public float FrictionCircleStrength = 0.75f;


        /// <summary>
        /// Number of sides the cylinder used for wheel cast will have.
        /// Minimum adequate value for the use case is recommended as this has a high impact on performance.
        /// </summary>
        [Header("Wheel Collider")]
        [UnityEngine.Tooltip("Number of sides the cylinder used for wheel cast will have. Minimum adequate value for the use case is recommended as this has a high impact on performance.")]
        public int WheelColliderSideCount = 6;

        /// <summary>
        /// Layer overrides for the generated wheel collider.
        /// This will affect both wheel collider cast queries, and wheel protector contacts generation if the
        /// <see cref="AddWheelProtectorCollider"/> option is enabled.
        /// </summary>
        [UnityEngine.Tooltip("Layer overrides for the generated wheel collider. This will affect both wheel collider cast queries, and wheel protector contacts generation if the 'AddWheelProtectorColliders' option is enabled.")]
        public WheelColliderLayerOverrides WheelColliderLayerOverrides;

        /// <summary>
        /// Should the wheel protector collider for this wheel be added to the vehicle rigidbody?
        /// This collider represents the wheel at full suspension compression, and is used to handle
        /// suspension bottoming out and impacts from directions that are not handled by the suspension.
        /// </summary>
        [UnityEngine.Tooltip("Should the wheel protector collider for this wheel be added to the vehicle rigidbody? This collider represents the wheel at full suspension compression, and is used to handle suspension bottoming out and impacts from directions that are not handled by the suspension.")]
        public bool AddWheelProtectorCollider = true;

        /// <summary>
        /// Whether or not the wheel collider provides contacts.
        /// This only matters if the <see cref="AddWheelProtectorCollider"/> option is enabled.
        /// </summary>
        [UnityEngine.Tooltip("Whether or not the wheel collider provides contacts. This only matters if the 'AddWheelProtectorColliders' option is enabled.")]
        public bool WheelProtectorColliderProvidesContacts = false;

        /// <summary>
        /// Physics material for the generated wheel collider.
        /// This only matters if <see cref="AddWheelProtectorCollider"/> option is enabled.
        /// </summary>
        [UnityEngine.Tooltip("Physics material for the generated wheel collider. This only matters if the 'AddWheelProtectorColliders' option is enabled.")]
        public UnityEngine.PhysicsMaterial WheelProtectorColliderPhysicsMaterial;

        private class Baker : Baker<WheelAuthoring>
        {
            static UnityEngine.PhysicsMaterial _defaultPhysicsMaterial;

            public override void Bake(WheelAuthoring authoring)
            {
                bool wheelHasRigidbody = authoring.GetComponent<Rigidbody>() != null;

                if (!wheelHasRigidbody)
                {
                    // If there are any colliders in wheel hierarchy but no rigidbody, warn about needing a rigidbody.
                    // Without a rigidbody, wheel colliders would become part of the vehicle compound collider.
                    UnityEngine.Collider[] collidersInWheel = authoring.gameObject.GetComponentsInChildren<UnityEngine.Collider>();
                    if (collidersInWheel != null && collidersInWheel.Length > 0)
                    {
                        Debug.LogError(
                            $"Colliders have been detected under the wheel object ({authoring.gameObject.name}). You must ensure that the wheel GameObject ({authoring.gameObject.name}) has a Rigidbody component.). Wheel baking aborted.");
                        return;
                    }
                }

                Entity entity = GetEntity(authoring, wheelHasRigidbody ? TransformUsageFlags.Dynamic : TransformUsageFlags.ManualOverride);

                ValidateSpinObject(authoring);

                if (!wheelHasRigidbody)
                {
                    // Transform components
                    AddComponent(entity, LocalTransform.FromPositionRotationScale(
                        authoring.transform.position,
                        authoring.transform.rotation,
                        1f));
                    AddComponent(entity, new LocalToWorld
                    {
                        Value = authoring.transform.localToWorldMatrix,
                    });
                }

                Wheel wheel = new Wheel
                {
                    SpinEntity = GetEntity(authoring.WheelSpinObject, TransformUsageFlags.Dynamic),
                    Radius = math.clamp(authoring.Radius, SMALL_VALUE, math.INFINITY),
                    Width = math.clamp(authoring.Width, SMALL_VALUE, math.INFINITY),
                    Mass = math.clamp(authoring.Mass, SMALL_VALUE, math.INFINITY),

                    SuspensionMaxLength = math.clamp(authoring.SuspensionMaxLength, 0, math.INFINITY),
                    SuspensionSpringRate = math.clamp(authoring.SuspensionSpringRate, 0, math.INFINITY),
                    SuspensionDampingRate = math.clamp(authoring.SuspensionDampingRate, 0, math.INFINITY),
                    VisualSuspensionSharpness = math.clamp(authoring.VisualSuspensionSharpness, 0, math.INFINITY),
                    AntiRollBarStiffness = math.clamp(authoring.AntiRollBarStiffness, 0, math.INFINITY),

                    FrictionCurveMaximum = authoring.FrictionCurve.FrictionCurveMaximum,
                    FrictionCurveMinimum = authoring.FrictionCurve.FrictionCurveMinimum,
                    FrictionCircleStrength = math.clamp(authoring.FrictionCircleStrength, 0, 1),

                    AddWheelProtectorColliders = authoring.AddWheelProtectorCollider,
                };

                // Wheel colliders generation
                {
                    // Generate wheel collider cylinder
                    NativeArray<float3> cylinderPointsArray =
                        new NativeArray<float3>(authoring.WheelColliderSideCount * 2, Allocator.Temp);
                    MathUtilities.GenerateCylinderPoints(ref cylinderPointsArray, authoring.WheelColliderSideCount,
                        wheel.Radius, wheel.Width);

                    // Create the wheel collider physics material
                    Unity.Physics.Material wheelColliderPhysicsMaterial = new Unity.Physics.Material();
                    {
                        wheelColliderPhysicsMaterial.CollisionResponse = authoring.WheelProtectorColliderProvidesContacts
                            ? CollisionResponsePolicy.CollideRaiseCollisionEvents
                            : CollisionResponsePolicy.Collide;

                        UnityEngine.PhysicsMaterial referencePhysicsMaterial =
                            authoring.WheelProtectorColliderPhysicsMaterial;
                        if (referencePhysicsMaterial == null)
                        {
                            if (_defaultPhysicsMaterial == null)
                            {
                                _defaultPhysicsMaterial = new UnityEngine.PhysicsMaterial
                                { hideFlags = HideFlags.DontSave };
                            }

                            referencePhysicsMaterial = _defaultPhysicsMaterial;
                        }

                        wheelColliderPhysicsMaterial.Friction = referencePhysicsMaterial.dynamicFriction;
                        wheelColliderPhysicsMaterial.Restitution = referencePhysicsMaterial.bounciness;
                        switch (referencePhysicsMaterial.frictionCombine)
                        {
                            case PhysicsMaterialCombine.Average:
                                wheelColliderPhysicsMaterial.FrictionCombinePolicy =
                                    Material.CombinePolicy.ArithmeticMean;
                                break;
                            case PhysicsMaterialCombine.Minimum:
                                wheelColliderPhysicsMaterial.FrictionCombinePolicy =
                                    Material.CombinePolicy.Minimum;
                                break;
                            case PhysicsMaterialCombine.Maximum:
                                wheelColliderPhysicsMaterial.FrictionCombinePolicy =
                                    Material.CombinePolicy.Maximum;
                                break;
                            default:
                                wheelColliderPhysicsMaterial.FrictionCombinePolicy =
                                    Material.CombinePolicy.ArithmeticMean;
                                Debug.LogWarning(
                                    $"Wheel object {authoring.gameObject.name} uses unsupported friction combine policy {referencePhysicsMaterial.frictionCombine} for its wheel collider physics material. Defaulting to \"Average\".");
                                break;
                        }

                        switch (referencePhysicsMaterial.bounceCombine)
                        {
                            case PhysicsMaterialCombine.Average:
                                wheelColliderPhysicsMaterial.RestitutionCombinePolicy =
                                    Material.CombinePolicy.ArithmeticMean;
                                break;
                            case PhysicsMaterialCombine.Minimum:
                                wheelColliderPhysicsMaterial.RestitutionCombinePolicy =
                                    Material.CombinePolicy.Minimum;
                                break;
                            case PhysicsMaterialCombine.Maximum:
                                wheelColliderPhysicsMaterial.RestitutionCombinePolicy =
                                    Material.CombinePolicy.Maximum;
                                break;
                            default:
                                wheelColliderPhysicsMaterial.RestitutionCombinePolicy =
                                    Material.CombinePolicy.ArithmeticMean;
                                Debug.LogWarning(
                                    $"Wheel object {authoring.gameObject.name} uses unsupported bounce combine policy {referencePhysicsMaterial.bounceCombine} for its wheel protector collider physics material. Defaulting to \"Average\".");
                                break;
                        }
                    }

                    // Wheel collider collision filter
                    CollisionFilter wheelColliderCollisionFilter = CollisionFilter.Default;
                    {
                        // Declaring the dependency on the GameObject with GetLayer, so the baker rebakes if the layer changes
                        int layer = GetLayer(authoring);

                        // create filter and assign layer of this collider
                        wheelColliderCollisionFilter = new CollisionFilter { BelongsTo = 1u << layer };

                        uint includeMask = 0u;
                        // incorporate global layer collision matrix
                        for (var i = 0; i < 32; ++i)
                        {
                            includeMask |= UnityEngine.Physics.GetIgnoreLayerCollision(layer, i) ? 0 : 1u << i;
                        }

                        // Now incorporate the layer overrides.
                        // The exclude layers take precedence over the include layers.

                        includeMask |= (uint)authoring.WheelColliderLayerOverrides.IncludeLayers.value;
                        var excludeMask = (uint)authoring.WheelColliderLayerOverrides.ExcludeLayers.value;

                        // apply exclude mask to include mask and set the final result
                        includeMask &= ~excludeMask;

                        wheelColliderCollisionFilter.CollidesWith = includeMask;
                    }

                    BlobAssetReference<Physics.Collider> wheelColliderReference =
                        ConvexCollider.Create(
                            cylinderPointsArray,
                            ConvexHullGenerationParameters.Default,
                            wheelColliderCollisionFilter,
                            wheelColliderPhysicsMaterial);

                    AddBlobAsset(ref wheelColliderReference, out var hash);
                    wheel.WheelCollider = wheelColliderReference;

                    cylinderPointsArray.Dispose();
                }

                // Check the friction curve for invalid data
                if (wheel.FrictionCurveMinimum.x < wheel.FrictionCurveMaximum.x)
                {
                    wheel.FrictionCurveMinimum.x = wheel.FrictionCurveMaximum.x + math.EPSILON;
                }

                wheel.FrictionCurveMaximum.x = math.saturate(wheel.FrictionCurveMaximum.x);
                wheel.FrictionCurveMinimum.x = math.saturate(wheel.FrictionCurveMinimum.x);
                wheel.FrictionCurveMaximum.y = math.clamp(wheel.FrictionCurveMaximum.y, 0, math.INFINITY);
                wheel.FrictionCurveMinimum.y = math.clamp(wheel.FrictionCurveMinimum.y, 0, math.INFINITY);

                AddComponent(entity, wheel);
                AddComponent(entity, new WheelOnVehicleReference
                {
                    VehicleEntity = Entity.Null,
                    Index = -1,
                });
            }

            private bool ValidateSpinObject(WheelAuthoring authoring)
            {
                if (authoring.WheelSpinObject == null)
                    return true;

                bool isDirectChildOfWheel = authoring.WheelSpinObject.transform.parent == authoring.transform;

                if (!isDirectChildOfWheel)
                {
                    UnityEngine.Debug.LogError(
                        $"The SpinObject {authoring.WheelSpinObject} of wheel {authoring.gameObject} must be a direct child of the wheel transform");
                    return false;
                }

                return true;
            }
        }
    }
}