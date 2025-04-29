using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;

namespace Unity.Vehicles
{
    /// <summary>
    /// System that handles physics contact modification for vehicles, allowing smoother collisions
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(PhysicsCreateContactsGroup))]
    [UpdateBefore(typeof(PhysicsCreateJacobiansGroup))]
    public partial struct VehicleContactModificationSystem : ISystem
    {
        [BurstCompile]
        internal void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsWorldSingleton>();
            state.RequireForUpdate<Vehicle>();
        }

        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            state.Dependency = new ModifyContactNormalsJob
            {
                PhysicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld,
                VehicleLookup = SystemAPI.GetComponentLookup<Vehicle>(true),
            }.Schedule(
                SystemAPI.GetSingletonRW<SimulationSingleton>().ValueRW,
                ref SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld,
                state.Dependency);
        }

        /// <summary>
        /// Job for modifying vehicle contacts
        /// </summary>
        [BurstCompile]
        struct ModifyContactNormalsJob : IContactsJob
        {
            [ReadOnly] public ComponentLookup<Vehicle> VehicleLookup;
            [ReadOnly] public PhysicsWorld PhysicsWorld;

            float m_DistanceScale;

            public unsafe void Execute(ref ModifiableContactHeader contactHeader,
                ref ModifiableContactPoint contactPoint)
            {
                if ((contactHeader.JacobianFlags & JacobianFlags.IsTrigger) == 0)
                {
                    // TODO: faster way to check vehicle with tag
                    bool vehicleIsBodyA = VehicleLookup.HasComponent(contactHeader.EntityA);
                    bool vehicleIsBodyB = VehicleLookup.HasComponent(contactHeader.EntityB);
                    if (vehicleIsBodyA != vehicleIsBodyB) // conditional xor
                    {
                        var vehicleEntity = vehicleIsBodyA ? contactHeader.EntityA : contactHeader.EntityB;
                        VehicleLookup.TryGetComponent(vehicleEntity, out var vehicle);
                        if (!vehicle.UseContactFiltering)
                            return;

                        var meshEntity = vehicleIsBodyA ? contactHeader.EntityB : contactHeader.EntityA;
                        int meshBodyIndex = PhysicsWorld.GetRigidBodyIndex(meshEntity);

                        RigidBody meshBody = PhysicsWorld.Bodies[meshBodyIndex];
                        if (meshBody.Collider.Value.CollisionType != CollisionType.Composite)
                            return;

                        // check if a new contact batch is starting
                        if (contactPoint.Index == 0)
                        {
                            // reset distance scale for this batch of contacts
                            m_DistanceScale = 1f;

                            // if we have a mesh surface we can get the surface normal from the plane of the polygon
                            meshBody.Collider.Value.GetLeaf(
                                vehicleIsBodyA ? contactHeader.ColliderKeyB : contactHeader.ColliderKeyA,
                                out ChildCollider hitLeafCollider);

                            if (hitLeafCollider.Collider->Type == ColliderType.Triangle ||
                                hitLeafCollider.Collider->Type == ColliderType.Quad)
                            {
                                PolygonCollider* polygonCollider = (PolygonCollider*)hitLeafCollider.Collider;

                                quaternion polygonRotation = math.mul(meshBody.WorldFromBody.rot,
                                    hitLeafCollider.TransformFromChild.rot);
                                float3 polygonSurfaceNormal =
                                    math.rotate(polygonRotation, polygonCollider->Planes[0].Normal);

                                // Check if the surface normal should be filtered, by confirming that it is part of the terrain "surface",
                                // and not a feature such as a wall or front face of a stair step, which we do want the chassis to collide
                                // with as usual. For this test we are using the vehicle chassis' up direction.

                                // get linear velocity at contact point to check if it would be beneficial to change the contact normal
                                int vehicleBodyIndex = PhysicsWorld.GetRigidBodyIndex(vehicleEntity);
                                MotionVelocity vehicleVelocity = PhysicsWorld.MotionVelocities[vehicleBodyIndex];
                                MotionData vehicleMotionData = PhysicsWorld.MotionDatas[vehicleBodyIndex];
                                var angularVelocityWorldSpace = math.rotate(vehicleMotionData.WorldFromMotion.rot,
                                    vehicleVelocity.AngularVelocity);
                                var linearVelocityAtContactPointWorld = vehicleVelocity.LinearVelocity +
                                                                        math.cross(angularVelocityWorldSpace,
                                                                            contactPoint.Position -
                                                                            vehicleMotionData.WorldFromMotion.pos);
                                var sqrLength = math.lengthsq(linearVelocityAtContactPointWorld);
                                if (sqrLength > math.EPSILON)
                                {
                                    var length = math.sqrt(sqrLength);
                                    var velDir = linearVelocityAtContactPointWorld / length;
                                    
                                    var cosFeatureAngle = math.dot(-velDir, polygonSurfaceNormal);
                                    float kCosFeatureAngle = vehicle.HardTerrainFeatureAngleThresholdCos;
                                    if (cosFeatureAngle > kCosFeatureAngle)
                                    {
                                        //Debug.Log("Feature detected. Skip.");
                                        return;
                                    }
                                }

                                // now filter contact based on the angle between its normal and the terrain surface normal  

                                // Flip the contact normal if the vehicle is body B, since by convention the normal
                                // always points towards body A, and we must have it point towards the vehicle chassis
                                // for the math to work.
                                var contactNormal = math.select(-contactHeader.Normal, contactHeader.Normal,
                                    vehicleIsBodyA);
                                var normalCosAngle = math.dot(polygonSurfaceNormal, contactNormal);
                                float kCosAngle = vehicle.ContactSurfaceNormalAngleThresholdCos;

                                if (normalCosAngle > kCosAngle || normalCosAngle < 1f - kCosAngle)
                                {
                                    return;
                                }

                                m_DistanceScale = normalCosAngle;
                                contactHeader.Normal = polygonSurfaceNormal;
                            }
                        }

                        if (m_DistanceScale < 1f)
                        {
                            contactPoint.Distance *= m_DistanceScale;
                        }
                    }
                }
            }
        }
    }
}