using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using UnityEngine;

namespace Unity.Vehicles
{
    /// <summary>
    /// Handles vehicles debug drawing.
    /// </summary>
    [ExecuteAlways]
    public class VehicleDebug : MonoBehaviour
    {
        /// <summary>
        /// Should the wheel axis/directions be drawn?
        /// </summary>
        [Header("Wheel Axis")]
        [Tooltip("Should the wheel axis/directions be drawn?")]
        public bool DrawWheelAxis = true;

        /// <summary>
        /// Length of the wheel axis/directions.
        /// </summary>
        [Tooltip("Length of the wheel axis/directions.")]
        public float WheelAxisLength = 1f;

        /// <summary>
        /// Should the wheel casting results be drawn?
        /// </summary>
        [Header("Wheel Hits")]
        [Tooltip("Should the wheel casting results be drawn?")]
        public bool DrawWheelCasts = true;

        /// <summary>
        /// Color of the wheel cast positions and normals.
        /// </summary>
        [Tooltip("Color of the wheel cast positions and normals.")]
        public Color WheelCastColor = Color.red;

        /// <summary>
        /// Length of the wheel casts.
        /// </summary>
        [Tooltip("Length of the wheel casts.")]
        public float WheelHitsLength = 1f;

        /// <summary>
        /// Should the wheel gizmos be drawn?
        /// </summary>
        [Header("Wheels")]
        [Tooltip("Should the wheel gizmos be drawn?")]
        public bool DrawWheels = true;

        /// <summary>
        /// Should the friction gizmos be drawn?
        /// </summary>
        [Header("Friction")]
        [Tooltip("Should the friction gizmos be drawn?")]
        public bool DrawFriction = true;

        /// <summary>
        /// Color of the friction force gizmos.
        /// </summary>
        [Tooltip("Color of the friction force gizmos.")]
        public Color FrictionForceColor = Color.white;

        /// <summary>
        /// Scale of the friction force gizmos.
        /// </summary>
        [Tooltip("Scale of the friction force gizmos.")]
        public float FrictionForceScale = 1e-5f;

        /// <summary>
        /// Color of the friction slip gizmos.
        /// </summary>
        [Tooltip("Color of the friction slip gizmos.")]
        public Color FrictionSlipColor = Color.yellow;

        /// <summary>
        /// Scale of the friction slip gizmos.
        /// </summary>
        [Tooltip("Scale of the friction slip gizmos.")]
        public float FrictionSlipScale = 2f;

        /// <summary>
        /// Color of the friction speed gizmos.
        /// </summary>
        [Tooltip("Color of the friction speed gizmos.")]
        public Color FrictionSpeedColor = Color.magenta;

        /// <summary>
        /// Scale of the friction speed gizmos.
        /// </summary>
        [Tooltip("Scale of the friction speed gizmos.")]
        public float FrictionSpeedScale = 0.02f;

        /// <summary>
        /// Should the suspension force gizmos be drawn?
        /// </summary>
        [Header("Suspension")]
        [Tooltip("Should the suspension force gizmos be drawn?")]
        public bool DrawSuspensionForces = true;

        /// <summary>
        /// Color of the suspension force gizmos.
        /// </summary>
        [Tooltip("Color of the suspension force gizmos.")]
        public Color SuspensionForceColor = Color.grey;

        /// <summary>
        /// Scale of the suspension force gizmos.
        /// </summary>
        [Tooltip("Scale of the suspension force gizmos.")]
        public float SuspensionForceScale = 1e-5f;

        /// <summary>
        /// Should the aero gizmos be drawn?
        /// </summary>
        [Header("Aero")]
        [Tooltip("Should the aero gizmos be drawn?")]
        public bool DrawAeroForces = true;

        /// <summary>
        /// Color of the aero force gizmos.
        /// </summary>
        [Tooltip("Color of the aero force gizmos.")]
        public Color AeroForceColor = Color.cyan;

        /// <summary>
        /// Scale of the aero force gizmos.
        /// </summary>
        [Tooltip("Scale of the aero force gizmos.")]
        public float AeroForceScale = 1e-4f;

        private World _observedWorld = null;

        /// <summary>
        /// Sets the debug world
        /// </summary>
        /// <param name="world">ECS world to debug</param>
        public void SetObservedWorld(World world)
        {
            _observedWorld = world;
        }
        
        private unsafe void OnDrawGizmos()
        {
            // Gizmos that should be drawn out of play mode are handled by authoring
            if (!Application.isPlaying)
            {
                return;
            }

            World world = _observedWorld;
            if (world == null)
            {
                world = World.DefaultGameObjectInjectionWorld;
            }
            
            EntityManager entityManager = world.EntityManager;

            entityManager.CompleteAllTrackedJobs();

            EntityQuery vehiclesQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<Vehicle, VehicleControlData, WheelOnVehicle, LocalTransform, LocalToWorld, PhysicsMass>()
                .Build(entityManager);

            NativeArray<Entity> vehicleEntities = vehiclesQuery.ToEntityArray(Allocator.Temp);
            NativeArray<Vehicle> vehicles = vehiclesQuery.ToComponentDataArray<Vehicle>(Allocator.Temp);
            NativeArray<VehicleControlData> vehicleControlDatas =
                vehiclesQuery.ToComponentDataArray<VehicleControlData>(Allocator.Temp);
            NativeArray<LocalTransform> vehicleTransforms =
                vehiclesQuery.ToComponentDataArray<LocalTransform>(Allocator.Temp);
            NativeArray<LocalToWorld> vehicleLTWs = vehiclesQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            NativeArray<PhysicsMass> vehicleMasses = vehiclesQuery.ToComponentDataArray<PhysicsMass>(Allocator.Temp);

            for (int i = 0; i < vehicleEntities.Length; i++)
            {
                Entity entity = vehicleEntities[i];
                Vehicle vehicle = vehicles[i];
                VehicleControlData vehicleControlData = vehicleControlDatas[i];
                LocalTransform vehicleLocalTransform = vehicleTransforms[i];
                LocalToWorld vehicleLTW = vehicleLTWs[i];
                PhysicsMass mass = vehicleMasses[i];
                DynamicBuffer<WheelOnVehicle> vehicleWheels = entityManager.GetBuffer<WheelOnVehicle>(entity);

                if (DrawAeroForces)
                {
                    Gizmos.color = AeroForceColor;
                    Gizmos.DrawRay(vehicleControlData.AeroDragPosition, vehicleControlData.AeroDragImpulse * AeroForceScale);
                    Gizmos.DrawRay(vehicleControlData.AeroDownforcePosition,
                        vehicleControlData.AeroDownforceImpulse * AeroForceScale);
                }

                RigidTransform vehicleTransform =
                    new RigidTransform(vehicleLTW.Value.Rotation(), vehicleLTW.Value.Translation());

                for (int w = 0; w < vehicleWheels.Length; w++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheels[w];
                    Wheel wheel = wheelOnVehicle.Wheel;

                    // Updated (interpolated) wheel and suspension position in relation to the parent.
                    RigidTransform interpolatedSuspensionTransform =
                        VehicleUtilities.GetSteeredSuspensionWorldTransform(in vehicleTransform, in wheelOnVehicle);
                    RigidTransform interpolatedWheelTransform =
                        VehicleUtilities.GetWheelWorldTransform(in interpolatedSuspensionTransform, wheelOnVehicle.SuspensionLength);
                    float3 interpolatedSuspensionDirection = math.mul(interpolatedSuspensionTransform.rot, math.down());
                    float3 interpolatedCenteredThreadPoint = interpolatedWheelTransform.pos + interpolatedSuspensionDirection * wheelOnVehicle.Wheel.Radius;

                    // Wheel axis
                    if (DrawWheelAxis)
                    {
                        Gizmos.color = Color.blue;
                        Gizmos.DrawRay(interpolatedWheelTransform.pos,
                            VehicleUtilities.GetWheelForward(in interpolatedSuspensionTransform) * WheelAxisLength);
                        Gizmos.color = Color.green;
                        Gizmos.DrawRay(interpolatedWheelTransform.pos,
                            VehicleUtilities.GetWheelRight(in interpolatedSuspensionTransform) * WheelAxisLength);
                        Gizmos.color = Color.red;
                        Gizmos.DrawRay(interpolatedWheelTransform.pos,
                            VehicleUtilities.GetWheelUp(in interpolatedSuspensionTransform) * WheelAxisLength);
                    }
                    
                    // Wheel cast
                    if (DrawWheelCasts)
                    {
                        Gizmos.color = WheelCastColor;
                        Gizmos.DrawWireSphere(wheelOnVehicle.WheelHit.Position, wheel.Radius * 0.04f);

                        if (wheelOnVehicle.WheelHit.Entity != Entity.Null)
                        {
                            Gizmos.DrawRay(wheelOnVehicle.WheelHit.Position, wheelOnVehicle.WheelHit.SurfaceNormal * WheelHitsLength);
                        }
                    }
                    
                    // Wheels
                    if (DrawWheels)
                    {
                        VehicleUtilities.DrawWheelGizmo(
                            interpolatedSuspensionTransform,
                            interpolatedWheelTransform,
                            wheel.Radius,
                            wheel.Width,
                            wheel.SuspensionMaxLength);
                    }
                    
                    // Friction
                    if (DrawFriction)
                    {
                        // Draw forces
                        Gizmos.color = FrictionForceColor;
                        Gizmos.DrawRay(interpolatedCenteredThreadPoint,
                            wheelOnVehicle.FrictionImpulse * FrictionForceScale);

                        //  Draw slip
                        Gizmos.color = FrictionSlipColor;
                        Gizmos.DrawRay(interpolatedCenteredThreadPoint,
                            VehicleUtilities.GetWheelForward(in interpolatedSuspensionTransform) * wheelOnVehicle.FrictionSlip.x * FrictionSlipScale);
                        Gizmos.DrawRay(interpolatedCenteredThreadPoint,
                            VehicleUtilities.GetWheelRight(in interpolatedSuspensionTransform) * wheelOnVehicle.FrictionSlip.y * FrictionSlipScale);

                        // Draw friction speed components
                        Gizmos.color = FrictionSpeedColor;
                        Gizmos.DrawRay(interpolatedCenteredThreadPoint,
                            VehicleUtilities.GetWheelForward(in interpolatedSuspensionTransform) * wheelOnVehicle.FrictionSpeed.x * FrictionSpeedScale);
                        Gizmos.DrawRay(interpolatedCenteredThreadPoint,
                            VehicleUtilities.GetWheelRight(in interpolatedSuspensionTransform) * wheelOnVehicle.FrictionSpeed.y * FrictionSpeedScale);
                    }

                    // Suspension
                    if (DrawSuspensionForces)
                    {
                        Gizmos.color = SuspensionForceColor;
                        Gizmos.DrawRay(interpolatedSuspensionTransform.pos,
                            wheelOnVehicle.SuspensionImpulse * SuspensionForceScale);
                    }
                }
            }

            vehicleEntities.Dispose();
            vehicles.Dispose();
            vehicleControlDatas.Dispose();
            vehicleTransforms.Dispose();
            vehicleLTWs.Dispose();
            vehicleMasses.Dispose();
        }
    }
}