using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;
using Collider = Unity.Physics.Collider;

namespace Unity.Vehicles
{
    /// <summary>
    /// System handling the update of the vehicle compound collider, which includes the vehicle body collider and
    /// wheel protector colliders if any
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(BeforePhysicsSystemGroup))]
    public partial struct VehicleCollidersSystem : ISystem
    {
        private NativeHashMap<Entity, BlobAssetReference<Collider>> _createdVehicleColliders;

        [BurstCompile]
        internal void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<VehicleUpdateColliders, PhysicsCollider>().Build());

            _createdVehicleColliders =
                new NativeHashMap<Entity, BlobAssetReference<Collider>>(128, Allocator.Persistent);
        }

        [BurstCompile]
        internal void OnDestroy(ref SystemState state)
        {
            // Dispose all colliders created at runtime
            NativeKeyValueArrays<Entity, BlobAssetReference<Collider>> createdColliders =
                _createdVehicleColliders.GetKeyValueArrays(Allocator.Temp);
            for (int i = 0; i < createdColliders.Keys.Length; i++)
            {
                BlobAssetReference<Collider> collider = createdColliders.Values[i];
                if (collider.IsCreated)
                {
                    collider.Dispose();
                }
            }

            createdColliders.Dispose();

            _createdVehicleColliders.Dispose();
        }

        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            EntityQuery uninitializedVehiclesQuery =
                SystemAPI.QueryBuilder().WithAll<Vehicle>().WithNone<VehicleColliderCleanup>().Build();
            EntityQuery destroyedVehiclesQuery =
                SystemAPI.QueryBuilder().WithAll<VehicleColliderCleanup>().WithNone<Vehicle>().Build();
            EntityQuery vehicleCollidersToUpdateQuery =
                SystemAPI.QueryBuilder().WithAll<Vehicle, VehicleUpdateColliders, PhysicsCollider, WheelOnVehicle>().Build();

            // Initialize
            int uninitializedVehiclesCount = uninitializedVehiclesQuery.CalculateEntityCount();
            if (uninitializedVehiclesCount > 0)
            {
                NativeArray<VehicleColliderCleanup> components =
                    new NativeArray<VehicleColliderCleanup>(uninitializedVehiclesCount, Allocator.Temp);
                state.EntityManager.AddComponentData(uninitializedVehiclesQuery, components);
                components.Dispose();
            }

            // Destroy
            int destroyedVehiclesCount = destroyedVehiclesQuery.CalculateEntityCount();
            if (destroyedVehiclesCount > 0)
            {
                foreach (var (vehicleCleanup, entity) in SystemAPI.Query<VehicleColliderCleanup>().WithNone<Vehicle>()
                             .WithEntityAccess())
                {
                    DisposeColliderForEntity(entity, ref _createdVehicleColliders);
                }

                state.EntityManager.RemoveComponent<VehicleColliderCleanup>(destroyedVehiclesQuery);
            }

            // Update colliders
            if (vehicleCollidersToUpdateQuery.CalculateEntityCount() > 0)
            {
                vehicleCollidersToUpdateQuery.CompleteDependency();

                foreach (var (updateColliders, localTransform, vehicle,
                             physicsCollider, vehicleWheels, entity) in
                         SystemAPI
                             .Query<EnabledRefRW<VehicleUpdateColliders>, RefRO<LocalTransform>, RefRW<Vehicle>,
                                 RefRW<PhysicsCollider>, DynamicBuffer<WheelOnVehicle>>()
                             .WithAll<VehicleColliderCleanup>()
                             .WithEntityAccess())
                {
                    // Calculate how many wheel colliders are needed
                    int wheelProtectorCollidersCount = 0;
                    DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer = vehicleWheels;
                    for (int i = 0; i < vehicleWheelsBuffer.Length; i++)
                    {
                        WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[i];
                        if (wheelOnVehicle.Wheel.AddWheelProtectorColliders)
                        {
                            wheelProtectorCollidersCount++;
                        }
                    }

                    // Dispose current created colliders for this entity if any
                    DisposeColliderForEntity(entity, ref _createdVehicleColliders);

                    // Create new compound collider that includes vehicle body and wheels
                    {
                        int childColliderCounter = 0;
                        NativeArray<CompoundCollider.ColliderBlobInstance> allColliders =
                            new NativeArray<CompoundCollider.ColliderBlobInstance>(wheelProtectorCollidersCount + 1,
                                Allocator.Temp);

                        // Vehicle body
                        allColliders[childColliderCounter] = new CompoundCollider.ColliderBlobInstance
                        {
                            CompoundFromChild =
                                new RigidTransform(quaternion.identity,
                                    float3.zero), // TODO; should this always be identity transform?
                            Collider = vehicle.ValueRO.BodyCollider,
                            Entity = entity,
                        };
                        childColliderCounter++;

                        // Wheels
                        for (int i = 0; i < vehicleWheelsBuffer.Length; i++)
                        {
                            WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[i];
                            if (wheelOnVehicle.Wheel.AddWheelProtectorColliders)
                            {
                                allColliders[childColliderCounter] = new CompoundCollider.ColliderBlobInstance
                                {
                                    CompoundFromChild = new RigidTransform(wheelOnVehicle.SuspensionLocalTransform.rot,
                                        wheelOnVehicle.SuspensionLocalTransform.pos),
                                    Collider = wheelOnVehicle.Wheel.WheelCollider,
                                    Entity = wheelOnVehicle.Entity,
                                };

                                wheelOnVehicle.WheelProtectorChildColliderIndex = childColliderCounter;
                                childColliderCounter++;

                            }
                            vehicleWheelsBuffer[i] = wheelOnVehicle;
                        }

                        // Create and register compound collider
                        BlobAssetReference<Collider> compoundCollider = CompoundCollider.Create(allColliders);
                        physicsCollider.ValueRW.Value = compoundCollider;
                        _createdVehicleColliders.Add(entity, compoundCollider);

                        allColliders.Dispose();
                    }

                    updateColliders.ValueRW = false;
                }
            }
        }

        private static void DisposeColliderForEntity(Entity entity, ref NativeHashMap<Entity, BlobAssetReference<Collider>> createdVehicleColliders)
        {
            if (createdVehicleColliders.TryGetValue(entity, out BlobAssetReference<Collider> collider))
            {
                if (collider.IsCreated)
                {
                    collider.Dispose();
                }

                createdVehicleColliders.Remove(entity);
            }
        }
    }
}