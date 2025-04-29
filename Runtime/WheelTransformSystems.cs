using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Physics.GraphicsIntegration;
using Unity.Physics.Systems;
using UnityEngine;

namespace Unity.Vehicles
{
    /// <summary>
    /// Sets the wheel rotation around the axle.
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(TransformSystemGroup))]
    public partial struct WheelTransformSpinSystem : ISystem
    {
        [BurstCompile]
        internal void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<VehicleTimeSingleton>();
        }

        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            state.Dependency = new WheelTransformSpinJob
            {
                ElapsedTime = SystemAPI.Time.ElapsedTime,
                VehicleTimeSingleton = SystemAPI.GetSingleton<VehicleTimeSingleton>(),
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            }.Schedule(state.Dependency);
        }

        /// <summary>
        /// Job handling wheel rotation around the axle
        /// </summary>
        [BurstCompile]
        public partial struct WheelTransformSpinJob : IJobEntity
        {
            /// <summary>
            /// The elapsed time
            /// </summary>
            public double ElapsedTime;
            /// <summary>
            /// The vehicle time singleton
            /// </summary>
            public VehicleTimeSingleton VehicleTimeSingleton;
            /// <summary>
            /// Local transform component lookup
            /// </summary>
            public ComponentLookup<LocalTransform> LocalTransformLookup;

            void Execute(in Vehicle vehicle, ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer)
            {
                for (int i = 0; i < vehicleWheelsBuffer.Length; i++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[i];

                    if (LocalTransformLookup.HasComponent(wheelOnVehicle.Wheel.SpinEntity))
                    {
                        RefRW<LocalTransform> spinEntityLocalTransformRW =
                            LocalTransformLookup.GetRefRW(wheelOnVehicle.Wheel.SpinEntity);
                        if (spinEntityLocalTransformRW.IsValid)
                        {
                            float wheelRotationAngle = wheelOnVehicle.RotationAngle;

                            if (VehicleTimeSingleton.LastPhysicsUpdateDeltaTime > 0f && VehicleTimeSingleton.LastPhysicsUpdateTime > 0.0)
                            {
                                // Apply a smoothing to compensate for the frequency of physics updates that determine
                                // wheel rotation
                                float timeRatioInPhysicsUpdates = math.saturate(
                                    (float)(ElapsedTime - VehicleTimeSingleton.LastPhysicsUpdateTime) /
                                    VehicleTimeSingleton.LastPhysicsUpdateDeltaTime);

                                float estimatedPrevRotationAngle = wheelOnVehicle.RotationAngle -
                                                                   (VehicleTimeSingleton.LastPhysicsUpdateDeltaTime *
                                                                    wheelOnVehicle.AngularVelocity);

                                wheelRotationAngle = math.lerp(estimatedPrevRotationAngle,
                                    wheelOnVehicle.RotationAngle, timeRatioInPhysicsUpdates);
                            }

                            spinEntityLocalTransformRW.ValueRW.Rotation = quaternion.RotateX(wheelRotationAngle);
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// System handling the update of wheel transforms before physics
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(BeforePhysicsSystemGroup))]
    public partial struct WheelBeforePhysicsTransformSystem : ISystem
    {
        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            state.Dependency = new WheelTransformsUpdateFromSimulationJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            }.Schedule(state.Dependency);
        }
    }

    /// <summary>
    /// System handling the update of wheel transforms
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(SmoothRigidBodiesGraphicalMotion))]
    [UpdateBefore(typeof(ParentSystem))]
    public partial struct WheelTransformSystem : ISystem
    {
        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            state.Dependency = new WheelTransformsUpdateFromPresentationWithoutInterpolationJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            }.Schedule(state.Dependency);
            state.Dependency = new WheelTransformsUpdateFromPresentationWithInterpolationJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            }.Schedule(state.Dependency);
        }
    }

    /// <summary>
    /// Job handling the update of wheel transforms from simulation vehicle transform
    /// </summary>
    [BurstCompile]
    [WithAll(typeof(LocalTransform))]
    public partial struct WheelTransformsUpdateFromSimulationJob : IJobEntity
    {
        /// <summary>
        /// The time delta
        /// </summary>
        public float DeltaTime;
        /// <summary>
        /// Local transform component lookup
        /// </summary>
        public ComponentLookup<LocalTransform> LocalTransformLookup;

        void Execute(Entity entity, ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer)
        {
            LocalTransform vehicleLocalTransform = LocalTransformLookup[entity];
            RigidTransform vehicleTransform =
                new RigidTransform(vehicleLocalTransform.Rotation, vehicleLocalTransform.Position);

            VehicleUtilities.UpdateWheelTransforms(
                DeltaTime,
                in vehicleTransform,
                ref vehicleWheelsBuffer,
                ref LocalTransformLookup);
        }
    }

    /// <summary>
    /// Job handling the update of wheel transforms from interpolated vehicle transform
    /// </summary>
    [BurstCompile]
    [WithAll(typeof(PhysicsGraphicalSmoothing))]
    public partial struct WheelTransformsUpdateFromPresentationWithInterpolationJob : IJobEntity
    {
        /// <summary>
        /// The time delta
        /// </summary>
        public float DeltaTime;
        /// <summary>
        /// Local transform component lookup
        /// </summary>
        public ComponentLookup<LocalTransform> LocalTransformLookup;

        void Execute(in LocalToWorld ltw, ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer)
        {
            RigidTransform vehicleTransform = new RigidTransform(ltw.Value);

            VehicleUtilities.UpdateWheelTransforms(
                DeltaTime,
                in vehicleTransform,
                ref vehicleWheelsBuffer,
                ref LocalTransformLookup);
        }
    }

    /// <summary>
    /// Job handling the update of wheel transforms from interpolated vehicle transform
    /// </summary>
    [BurstCompile]
    [WithNone(typeof(PhysicsGraphicalSmoothing))]
    [WithAll(typeof(LocalTransform))]
    public partial struct WheelTransformsUpdateFromPresentationWithoutInterpolationJob : IJobEntity
    {
        /// <summary>
        /// The time delta
        /// </summary>
        public float DeltaTime;
        /// <summary>
        /// Local transform component lookup
        /// </summary>
        public ComponentLookup<LocalTransform> LocalTransformLookup;

        void Execute(Entity entity, ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer)
        {
            LocalTransform vehicleLocalTransform = LocalTransformLookup[entity];
            RigidTransform vehicleTransform =
                new RigidTransform(vehicleLocalTransform.Rotation, vehicleLocalTransform.Position);

            VehicleUtilities.UpdateWheelTransforms(
                DeltaTime,
                in vehicleTransform,
                ref vehicleWheelsBuffer,
                ref LocalTransformLookup);
        }
    }
}