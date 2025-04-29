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
            state.Dependency = new WheelTransformsJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                LocalToWorldLookup = SystemAPI.GetComponentLookup<LocalToWorld>(true),
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            }.ScheduleParallel(state.Dependency);
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
            state.Dependency = new WheelTransformsJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                LocalToWorldLookup = SystemAPI.GetComponentLookup<LocalToWorld>(true),
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            }.ScheduleParallel(state.Dependency);
        }
    }

    /// <summary>
    /// Job handling the update of wheel transforms
    /// </summary>
    [BurstCompile]
    public partial struct WheelTransformsJob : IJobEntity
    {
        /// <summary>
        /// The time delta
        /// </summary>
        public float DeltaTime;
        /// <summary>
        /// Local to world component lookup
        /// </summary>
        [ReadOnly] public ComponentLookup<LocalToWorld> LocalToWorldLookup;
        /// <summary>
        /// Local transform component lookup
        /// </summary>
        [NativeDisableParallelForRestriction] public ComponentLookup<LocalTransform> LocalTransformLookup;

        void Execute(Entity entity, ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer)
        {
            RigidTransform vehicleTransform = new RigidTransform(LocalToWorldLookup[entity].Value);

            for (int i = 0; i < vehicleWheelsBuffer.Length; i++)
            {
                WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[i];
                if (LocalTransformLookup.HasComponent(wheelOnVehicle.Entity))
                {
                    // Calculate new wheel position relative to the parent vehicleTransform but do not store the values
                    // as that is reserved for the physics step.
                    RigidTransform newSuspensionWorldTransform =
                        VehicleUtilities.GetSteeredSuspensionWorldTransform(in vehicleTransform, in wheelOnVehicle);

                    // SuspensionLength is the same between the physics updates, so update the wheel position just 
                    // from the new vehicle and suspension transforms.
                    wheelOnVehicle.VisualSuspensionLength = math.lerp(wheelOnVehicle.VisualSuspensionLength,
                        wheelOnVehicle.SuspensionLength,
                        MathUtilities.GetSharpnessInterpolant(wheelOnVehicle.Wheel.VisualSuspensionSharpness, DeltaTime));

                    RigidTransform newWheelWorldTransform =
                        VehicleUtilities.GetWheelWorldTransform(in newSuspensionWorldTransform,
                            wheelOnVehicle.VisualSuspensionLength);

                    // Update the visual wheel position
                    LocalTransformLookup[wheelOnVehicle.Entity] =
                        LocalTransform.FromPositionRotation(newWheelWorldTransform.pos, newWheelWorldTransform.rot);

                    vehicleWheelsBuffer[i] = wheelOnVehicle;
                }
            }
        }
    }
}