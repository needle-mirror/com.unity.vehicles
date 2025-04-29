using System;
using Unity.Burst;
using Unity.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using UnityEngine;
using Collider = Unity.Physics.Collider;
using Unity.Transforms;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Unity.Vehicles
{
    [TemporaryBakingType]
    internal struct VehiclePairedWheelsBaking : IBufferElementData
    {
        public Entity LeftWheel;
        public Entity RightWheel;
    }

    /// <summary>
    /// Represents a pair of wheels that share an axle
    /// </summary>
    [Serializable]
    public struct PairedWheels
    {
        /// <summary>
        /// The left wheel
        /// </summary>
        public GameObject LeftWheel;
        /// <summary>
        /// The right wheel
        /// </summary>
        public GameObject RightWheel;
    }

    /// <summary>
    /// Authoring component for vehicles
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [DisallowMultipleComponent]
    public class VehicleAuthoring : MonoBehaviour
    {
        /// <summary>
        /// Array holding the wheels belonging to this vehicle in the front left, front right, rear left, rear right, etc. order.
        /// </summary>
        [Tooltip(
            "Array holding the wheels belonging to this vehicle in the front left, front right, rear left, rear right, etc. order.")]
        public WheelAuthoring[] Wheels = new WheelAuthoring[0];

        /// <summary>
        /// Array of wheel pairs belonging to the same axle.
        /// A pair consists of a left and a right wheel, in that order.
        /// </summary>
        [Tooltip("Array of wheel pairs belonging to the same axle.  " +
                 "A pair consists of a left and a right wheel, in that order.")]
        public PairedWheels[] AxlePairedWheels = new PairedWheels[0];

        /// <summary>
        /// Number of slices the vehicle physics will slice one physics frame into.
        /// Higher values should be used to achieve stiffer friction and better high speed handling.
        /// </summary>
        [Range(4, 32)]
        [Tooltip("Number of slices the vehicle physics will slice one physics frame into. " +
                 "Higher values should be used to achieve stiffer friction and better high speed handling.")]
        public int SubstepCount = 12;

        /// <summary>
        /// Dimensions of the vehicle, expressed in m. Used to scale certain value thresholds in vehicle code.
        /// May also be used for aerodynamics calculations if the Vehicle Control component is used.
        /// </summary>
        [Tooltip(
            "Dimensions of the vehicle, expressed in m. Used to scale certain value thresholds in vehicle code. " +
            "May also be used for aerodynamics calculations if the Vehicle Control component is used.")]
        public float3 Dimensions = new float3(1.8f, 2f, 4.5f);

        /// <summary>
        /// Local center position of the Dimensions gizmo
        /// </summary>
        [Tooltip("Local center position of the Dimensions gizmo")]
        public float3 DimensionsDebugCenter = float3.zero;

        /// <summary>
        /// Enables contact filtering for smoothing collisions between the vehicle and mesh colliders representing
        /// a terrain surface.
        /// </summary>
        [Tooltip("Enables contact filtering for smoothing collisions between the vehicle and mesh colliders " +
                 "representing a terrain surface.")]
        public bool UseContactFiltering = true;

        /// <summary>
        /// Angle threshold for collision smoothing between the vehicle and mesh colliders.
        ///
        /// If, at a given contact point between the vehicle and a mesh collider, the angle between the contact normal
        /// and the mesh surface normal is larger than this threshold and does not exceed 90 degrees minus this threshold,
        /// the normal of the corresponding contact will be set to the surface normal, ensuring smooth collisions with the
        /// terrain surface.
        /// </summary>
        [Tooltip("Angle threshold for collision smoothing between the vehicle and mesh colliders.\n\n" +
                 "If, at a given contact point between the vehicle and a mesh collider, the angle between the contact normal " +
                 "and the mesh surface normal is larger than this threshold and does not exceed 90 degrees minus this threshold, " +
                 "the normal of the corresponding contact will be set to the surface normal, ensuring smooth collisions with the " +
                 "terrain surface.")]
        public float ContactSurfaceNormalAngleThreshold = 10f;

        /// <summary>
        /// Angle threshold for detecting hard terrain features, for which no smoothing should be applied.
        ///
        /// Corresponds to a threshold on the delta angle between the inverted vehicle's velocity at a given contact
        /// point and the colliding terrain feature's surface normal.
        /// Any surface feature leading to a delta angle that is larger than the threshold will be considered a hard terrain
        /// feature and contacts with this feature will not be filtered, ensuring collisions with hard terrain features
        /// are retained, enabling accurate collision response.
        /// </summary>
        [Tooltip("Angle threshold for detecting hard terrain features, for which no smoothing should be applied.\n\n" +
                 "Corresponds to a threshold on the delta angle between the inverted vehicle's velocity at a given contact " +
                 "point and the colliding terrain feature's surface normal. " +
                 "Any surface feature leading to a delta angle that is larger than the threshold will be considered a hard terrain " +
                 "feature and contacts with this feature will not be filtered, ensuring collisions with hard terrain features " +
                 "are retained, enabling accurate collision response.")]
        public float HardTerrainFeatureAngleThreshold = 60f;


#if UNITY_EDITOR
        private Vector3[] _dimensionsDebugLinePoints = new Vector3[24];
#endif

        /// <summary>
        /// Auto detect wheels
        /// </summary>
        public void OnAutoDetectWheels()
        {
            Wheels = gameObject.GetComponentsInChildren<WheelAuthoring>(false);
        }

        private class Baker : Baker<VehicleAuthoring>
        {
            public override void Bake(VehicleAuthoring authoring)
            {
                Entity entity = GetEntity(authoring, TransformUsageFlags.Dynamic);

                Vehicle vehicle = new Vehicle
                {
                    SubstepCount = math.clamp(authoring.SubstepCount, 1, 32),
                    Dimensions = new float3(
                        math.clamp(authoring.Dimensions.x, 1e-6f, math.INFINITY),
                        math.clamp(authoring.Dimensions.y, 1e-6f, math.INFINITY),
                        math.clamp(authoring.Dimensions.z, 1e-6f, math.INFINITY)
                    ),
                    VirtualScale = math.clamp(authoring.Dimensions.z / 4f, 0.01f, 1f),
                    UseContactFiltering = authoring.UseContactFiltering,
                    ContactSurfaceNormalAngleThresholdCos =
                        math.cos(math.TORADIANS * authoring.ContactSurfaceNormalAngleThreshold),
                    HardTerrainFeatureAngleThresholdCos =
                        math.cos(math.TORADIANS * authoring.HardTerrainFeatureAngleThreshold),
                };

                AddComponent(entity, vehicle);

                AddComponent(entity, new VehicleUpdateColliders());
                SetComponentEnabled<VehicleUpdateColliders>(entity, true);
                AddComponent(entity, new VehicleUpdateWheelData());
                SetComponentEnabled<VehicleUpdateWheelData>(entity, false);
                AddBuffer<VehicleWheelEvent>(entity);

                DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer = AddBuffer<WheelOnVehicle>(entity);
                for (int i = 0; i < authoring.Wheels.Length; i++)
                {
                    WheelAuthoring wheel = authoring.Wheels[i];
                    if (wheel != null)
                    {
                        RigidTransform suspensionLocalTransform = new RigidTransform(
                            math.mul(math.inverse(authoring.transform.localToWorldMatrix),
                                wheel.transform.localToWorldMatrix));
                        Entity wheelEntity = GetEntity(wheel.gameObject, TransformUsageFlags.None);

                        // Serialize basic wheel data in the VehicleWheel buffer (will be used later in VehicleBakingSystem)
                        WheelOnVehicle wheelOnVehicle = new WheelOnVehicle
                        {
                            Entity = wheelEntity,
                            SuspensionLocalTransform = suspensionLocalTransform,
                        };

                        vehicleWheelsBuffer.Add(wheelOnVehicle);
                    }
                }

                // Verify that no wheel appears multiple times in paired wheels
                bool addPairedWheelsBuffer = true;
                HashSet<GameObject> uniqueWheels = new HashSet<GameObject>();
                for (int i = 0; i < authoring.AxlePairedWheels.Length; i++)
                {
                    PairedWheels pairedWheels = authoring.AxlePairedWheels[i];

                    GameObject duplicateWheel = null;
                    if (uniqueWheels.Contains(pairedWheels.LeftWheel))
                    {
                        duplicateWheel = pairedWheels.LeftWheel;
                    }

                    if (uniqueWheels.Contains(pairedWheels.RightWheel))
                    {
                        duplicateWheel = pairedWheels.RightWheel;
                    }

                    if (duplicateWheel != null)
                    {
                        Debug.LogError(
                            $"Error: wheel GameObject {pairedWheels.LeftWheel} appears multiple times in the paired wheels buffer. " +
                            $"Each wheel can only appear once at most. Vehicle paired wheels baking aborted");
                        addPairedWheelsBuffer = false;
                    }

                    if (pairedWheels.LeftWheel != null)
                    {
                        uniqueWheels.Add(pairedWheels.LeftWheel);
                    }

                    if (pairedWheels.RightWheel != null)
                    {
                        uniqueWheels.Add(pairedWheels.RightWheel);
                    }
                }

                if (addPairedWheelsBuffer)
                {
                    DynamicBuffer<VehiclePairedWheelsBaking> pairedWheelsBuffer =
                        AddBuffer<VehiclePairedWheelsBaking>(entity);
                    for (int i = 0; i < authoring.AxlePairedWheels.Length; i++)
                    {
                        PairedWheels pairedWheels = authoring.AxlePairedWheels[i];

                        if (pairedWheels.LeftWheel != null && pairedWheels.RightWheel != null)
                        {
                            pairedWheelsBuffer.Add(new VehiclePairedWheelsBaking
                            {
                                LeftWheel = GetEntity(pairedWheels.LeftWheel, TransformUsageFlags.Dynamic),
                                RightWheel = GetEntity(pairedWheels.RightWheel, TransformUsageFlags.Dynamic),
                            });
                        }
                    }
                }
            }
        }

        private void OnDrawGizmosSelected()
        {
#if UNITY_EDITOR
            if (Wheels == null) return;

            Rigidbody rigidbody = transform.GetComponent<Rigidbody>();
            if (rigidbody != null)
            {
                // Center of mass
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(rigidbody.worldCenterOfMass, Dimensions.z * 0.01f);
            }

            // Dimensions
            {
                Color tmpColor = Color.yellow;
                tmpColor.a = 0.3f;
                Gizmos.color = tmpColor;

                RigidTransform dimensionsTransform =
                    new RigidTransform(transform.rotation, transform.TransformPoint(DimensionsDebugCenter));
                Vector3 extents = Dimensions * 0.5f;
                Vector3 v0 = math.transform(dimensionsTransform, -extents); // left-down-back
                Vector3 v1 =
                    math.transform(dimensionsTransform,
                        new float3(extents.x, -extents.y, -extents.z)); // right-down-back
                Vector3 v2 =
                    math.transform(dimensionsTransform, new float3(-extents.x, extents.y, -extents.z)); // left-up-back
                Vector3 v3 =
                    math.transform(dimensionsTransform, new float3(extents.x, extents.y, -extents.z)); // right-up-back
                Vector3 v4 =
                    math.transform(dimensionsTransform,
                        new float3(-extents.x, -extents.y, extents.z)); // left-down-front
                Vector3 v5 =
                    math.transform(dimensionsTransform,
                        new float3(extents.x, -extents.y, extents.z)); // right-down-front
                Vector3 v6 =
                    math.transform(dimensionsTransform, new float3(-extents.x, extents.y, extents.z)); // left-up-front
                Vector3 v7 = math.transform(dimensionsTransform, extents); // right-up-front

                _dimensionsDebugLinePoints[0] = v0;
                _dimensionsDebugLinePoints[1] = v1;
                _dimensionsDebugLinePoints[2] = v0;
                _dimensionsDebugLinePoints[3] = v2;
                _dimensionsDebugLinePoints[4] = v3;
                _dimensionsDebugLinePoints[5] = v1;
                _dimensionsDebugLinePoints[6] = v3;
                _dimensionsDebugLinePoints[7] = v2;
                _dimensionsDebugLinePoints[8] = v7;
                _dimensionsDebugLinePoints[9] = v6;
                _dimensionsDebugLinePoints[10] = v7;
                _dimensionsDebugLinePoints[11] = v5;
                _dimensionsDebugLinePoints[12] = v4;
                _dimensionsDebugLinePoints[13] = v6;
                _dimensionsDebugLinePoints[14] = v4;
                _dimensionsDebugLinePoints[15] = v5;
                _dimensionsDebugLinePoints[16] = v0;
                _dimensionsDebugLinePoints[17] = v4;
                _dimensionsDebugLinePoints[18] = v1;
                _dimensionsDebugLinePoints[19] = v5;
                _dimensionsDebugLinePoints[20] = v2;
                _dimensionsDebugLinePoints[21] = v6;
                _dimensionsDebugLinePoints[22] = v3;
                _dimensionsDebugLinePoints[23] = v7;

                Gizmos.DrawLineList(_dimensionsDebugLinePoints);
            }

            // Wheel gizmos
            for (int i = 0; i < Wheels.Length; i++)
            {
                WheelAuthoring wheel = Wheels[i];
                if (wheel != null)
                {
                    RigidTransform suspensionWorldTransform = new RigidTransform(wheel.transform.localToWorldMatrix);

                    VehicleUtilities.DrawWheelGizmo(
                        suspensionWorldTransform,
                        suspensionWorldTransform,
                        wheel.Radius,
                        wheel.Width,
                        wheel.SuspensionMaxLength);
                }
                else
                {
                    Debug.LogWarning("Trying to draw a null wheel.");
                }
            }
#endif
        }
    }

    [BurstCompile]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    [UpdateAfter(typeof(EndColliderBakingSystem))]
    partial struct VehicleBakingSystem : ISystem
    {
        [BurstCompile]
        public unsafe void OnUpdate(ref SystemState state)
        {
            NativeList<WheelOnVehicle> tmpWheels = new NativeList<WheelOnVehicle>(Allocator.Temp);
            ComponentLookup<Wheel> wheelLookup = SystemAPI.GetComponentLookup<Wheel>(true);
            BufferLookup<WheelOnVehicle> vehicleWheelBufferLookup = SystemAPI.GetBufferLookup<WheelOnVehicle>(false);
            ComponentLookup<WheelOnVehicleReference> vehicleWheelReferenceLookup =
                SystemAPI.GetComponentLookup<WheelOnVehicleReference>(false);
            ComponentLookup<VehicleUpdateWheelData> vehicleUpdateWheelDataLookup =
                SystemAPI.GetComponentLookup<VehicleUpdateWheelData>(false);
            ComponentLookup<VehicleUpdateColliders> vehicleUpdateCollidersLookup =
                SystemAPI.GetComponentLookup<VehicleUpdateColliders>(false);
            BufferLookup<VehicleWheelEvent> vehicleEventsBufferLookup =
                SystemAPI.GetBufferLookup<VehicleWheelEvent>(false);

            foreach (var (vehicle, vehicleCollider, vehicleWheelsBuffer, vehiclePairedWheelsBaking,
                         vehicleUpdateColliders, entity) in
                     SystemAPI
                         .Query<RefRW<Vehicle>, RefRW<PhysicsCollider>,
                             DynamicBuffer<WheelOnVehicle>, DynamicBuffer<VehiclePairedWheelsBaking>,
                             RefRW<VehicleUpdateColliders>>()
                         .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)
                         .WithEntityAccess())
            {
                DynamicBuffer<WheelOnVehicle> vehicleWheels = vehicleWheelsBuffer;

                // Remember vehicle body collider
                vehicle.ValueRW.BodyCollider = vehicleCollider.ValueRW.Value;

                // Remember basic wheel data and clear data in buffer
                tmpWheels.Clear();
                tmpWheels.AddRange(vehicleWheels.AsNativeArray());
                vehicleWheels.Clear();

                // Add initial wheels
                for (int i = 0; i < tmpWheels.Length; i++)
                {
                    WheelOnVehicle wheelOnVehicle = tmpWheels[i];

                    Entity wheelEntity = wheelOnVehicle.Entity;
                    RigidTransform wheelLocalTransform = wheelOnVehicle.SuspensionLocalTransform;

                    // Find paired wheel entity
                    Entity pairedWheelEntity = Entity.Null;
                    for (int j = 0; j < vehiclePairedWheelsBaking.Length; j++)
                    {
                        VehiclePairedWheelsBaking pairedWheels = vehiclePairedWheelsBaking[j];
                        if (wheelEntity == pairedWheels.LeftWheel)
                        {
                            pairedWheelEntity = pairedWheels.RightWheel;
                            break;
                        }
                        else if (wheelEntity == pairedWheels.RightWheel)
                        {
                            pairedWheelEntity = pairedWheels.LeftWheel;
                            break;
                        }
                    }

                    VehicleUtilities.TryAddWheel(
                        entity,
                        wheelEntity,
                        pairedWheelEntity,
                        wheelLocalTransform,
                        in wheelLookup,
                        ref vehicleWheelBufferLookup,
                        ref vehicleWheelReferenceLookup,
                        ref vehicleUpdateCollidersLookup,
                        ref vehicleUpdateWheelDataLookup,
                        ref vehicleEventsBufferLookup);
                }
            }

            tmpWheels.Dispose();
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(VehicleAuthoring))]
    public class VehicleEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            if (GUILayout.Button("Auto Detect Wheels"))
            {
                VehicleAuthoring vehicleAuthoring = target as VehicleAuthoring;
                vehicleAuthoring.OnAutoDetectWheels();
                EditorUtility.SetDirty(target);
            }
        }
    }
#endif
}