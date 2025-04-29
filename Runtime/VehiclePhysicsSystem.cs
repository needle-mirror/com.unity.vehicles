using System.Runtime.CompilerServices;
using Unity.Assertions;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;

namespace Unity.Vehicles
{
    /// <summary>
    /// Stores impulse data to be applied at a later time.
    /// Used for applying forces to other Rigidbodies from inside the SolveVehiclePhysicsJob.
    /// </summary>
    internal struct DeferredImpulse
    {
        /// <summary>
        /// Impulse vector.
        /// </summary>
        public float3 Impulse;

        /// <summary>
        /// Position of the impulse in world coordinates.
        /// </summary>
        public float3 Position;

        /// <summary>
        /// The entity to apply the impulse to.
        /// </summary>
        public Entity Entity;
    }
    
    /// <summary>
    /// Holds time data used by vehicle systems
    /// </summary>
    public struct VehicleTimeSingleton : IComponentData
    {
        /// <summary>
        /// Time of the last physics update.
        /// </summary>
        internal double LastPhysicsUpdateTime;

        /// <summary>
        /// Delta time of the last physics update.
        /// </summary>
        internal float LastPhysicsUpdateDeltaTime;
    }

    /// <summary>
    /// Handles core vehicle physics simulation.
    /// Powertrain and vehicle control are handled by <see cref="VehicleControlSystem"/>.
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    public partial struct VehiclePhysicsSystem : ISystem
    {
        [BurstCompile]
        internal void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsStep>();
            state.RequireForUpdate<VehicleTimeSingleton>();

            // Create the time singleton
            Entity timeSingleton = state.EntityManager.CreateEntity();
            state.EntityManager.AddComponentData(timeSingleton, new VehicleTimeSingleton
            {
                LastPhysicsUpdateTime = SystemAPI.Time.ElapsedTime,
                LastPhysicsUpdateDeltaTime = SystemAPI.Time.DeltaTime,
            });
        }

        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            ref VehicleTimeSingleton timeSingletonRef = ref SystemAPI.GetSingletonRW<VehicleTimeSingleton>().ValueRW;
            timeSingletonRef.LastPhysicsUpdateTime = SystemAPI.Time.ElapsedTime;
            timeSingletonRef.LastPhysicsUpdateDeltaTime = SystemAPI.Time.DeltaTime;

            
            EntityQuery vehiclesQuery = SystemAPI.QueryBuilder()
                .WithAll<Simulate>()
                .WithAll<LocalTransform, PhysicsMass, PhysicsVelocity>()
                .WithAll<Vehicle, WheelOnVehicle>()
                .Build();

            NativeStream deferredImpulsesStream =
                new NativeStream(vehiclesQuery.CalculateChunkCount(), state.WorldUpdateAllocator);

            UpdateWheelDataJob updateWheelDataJob = new UpdateWheelDataJob
            {
                WheelLookup = SystemAPI.GetComponentLookup<Wheel>(true),
            };
            state.Dependency = updateWheelDataJob.Schedule(state.Dependency);

            SolveVehiclePhysicsJob solveVehiclePhysicsJob = new SolveVehiclePhysicsJob
            {
                VehicleTimeSingleton = timeSingletonRef,
                Gravity = SystemAPI.GetSingleton<PhysicsStep>().Gravity,
                PhysicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld,
                DeferredImpulses = deferredImpulsesStream.AsWriter(),
            };
            state.Dependency = solveVehiclePhysicsJob.ScheduleParallel(state.Dependency);

            ApplyDeferredImpulsesJob applyDeferredImpulsesJob = new ApplyDeferredImpulsesJob
            {
                PhysicsVelocityLookup = SystemAPI.GetComponentLookup<PhysicsVelocity>(false),
                PhysicsMassLookup = SystemAPI.GetComponentLookup<PhysicsMass>(true),
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                DeferredImpulses = deferredImpulsesStream.AsReader(),
            };
            state.Dependency = applyDeferredImpulsesJob.Schedule(state.Dependency);

            deferredImpulsesStream.Dispose(state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        [WithAll(typeof(VehicleUpdateWheelData))]
        [WithAll(typeof(Vehicle))]
        internal partial struct UpdateWheelDataJob : IJobEntity
        {
            [ReadOnly] internal ComponentLookup<Wheel> WheelLookup;

            void Execute(ref DynamicBuffer<WheelOnVehicle> vehicleWheels,
                EnabledRefRW<VehicleUpdateWheelData> updateWheelData)
            {
                for (int i = 0; i < vehicleWheels.Length; i++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheels[i];

                    // Store wheel entity data into a buffer on the vehicle
                    WheelLookup.TryGetComponent(wheelOnVehicle.Entity, out wheelOnVehicle.Wheel);

                    vehicleWheels[i] = wheelOnVehicle;
                }

                updateWheelData.ValueRW = false;
            }
        }

        /// <summary>
        /// Job for solving vehicle and wheel physics
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct SolveVehiclePhysicsJob : IJobEntity, IJobEntityChunkBeginEnd
        {
            /// <summary>
            /// Time data for vehicle logic.
            /// </summary>
            public VehicleTimeSingleton VehicleTimeSingleton;

            /// <summary>
            /// World gravity vector.
            /// </summary>
            public float3 Gravity;

            /// <summary>
            /// Physics world the vehicle is simulating in.
            /// </summary>
            [ReadOnly] public PhysicsWorld PhysicsWorld;

            /// <summary>
            /// <see cref="DeferredImpulses"/> that will be applied to other Rigidbodies after the job is finished.
            /// </summary>
            [NativeDisableParallelForRestriction] [NativeDisableContainerSafetyRestriction]
            public NativeStream.Writer DeferredImpulses;

            [NativeDisableParallelForRestriction] [NativeDisableContainerSafetyRestriction]
            private NativeList<TmpVehicleWheelData> _tmpVehicleWheelDatas;

            void Execute(
                Entity entity,
                in LocalTransform transform,
                in PhysicsMass physicsMass,
                ref PhysicsVelocity physicsVelocity,
                ref Vehicle vehicle,
                ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer)
            {
                RigidTransform vehicleTransform = new RigidTransform(transform.Rotation, transform.Position);

                _tmpVehicleWheelDatas.Clear();
                _tmpVehicleWheelDatas.Resize(vehicleWheelsBuffer.Length, NativeArrayOptions.ClearMemory);

                // Cache wheel data and do wheel casts
                for (int wheelIndex = 0; wheelIndex < vehicleWheelsBuffer.Length; wheelIndex++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[wheelIndex];
                    TmpVehicleWheelData tmpVehicleWheelData = _tmpVehicleWheelDatas[wheelIndex];
                    Wheel wheel = wheelOnVehicle.Wheel;

                    // Clear wheel state
                    {
                        wheelOnVehicle.IsGrounded = default;
                        wheelOnVehicle.WheelHit = default;
                        wheelOnVehicle.SuspensionImpulse = default;
                        wheelOnVehicle.FrictionImpulse = default;
                        wheelOnVehicle.FrictionSpeed = default;
                    }

                    // Update wheel transforms starting from vehicle transform and steering
                    tmpVehicleWheelData.UpdateSuspensionTransform(in vehicleTransform, in wheelOnVehicle);

                    // Cache wheel properties 
                    wheel.Radius = wheel.Radius < math.EPSILON ? math.EPSILON : wheel.Radius;
                    tmpVehicleWheelData.InvRadius = 1f / wheel.Radius;

                    // A bit on the higher side of the calculation for the wheel shape itself, 
                    // but there is also the drivetrain, brake disc, etc.
                    tmpVehicleWheelData.MomentOfInertia = wheel.Mass * wheel.Radius * wheel.Radius;
                    tmpVehicleWheelData.MomentOfInertia =
                        tmpVehicleWheelData.MomentOfInertia < math.EPSILON
                            ? math.EPSILON
                            : tmpVehicleWheelData.MomentOfInertia;
                    tmpVehicleWheelData.InvMomentOfInertia = 1f / tmpVehicleWheelData.MomentOfInertia;

                    // Wheel cast
                    if (wheel.WheelCollider.IsCreated)
                    {
                        // Cast wheel collider down
                        float castDistance = wheel.SuspensionMaxLength;

                        // Always cast at least a small amount for the IsGrounded to have a chance to return true, even 
                        // without any suspension.
                        float minWheelCastDistance = wheel.Radius * 0.05f;
                        if (castDistance < minWheelCastDistance)
                        {
                            castDistance = minWheelCastDistance;
                        }

                        ColliderCastInput colliderCastInput = new ColliderCastInput(
                            wheel.WheelCollider,
                            tmpVehicleWheelData.SteeredSuspensionWorldTransform.pos,
                            tmpVehicleWheelData.SteeredSuspensionWorldTransform.pos +
                            tmpVehicleWheelData.SteeredSuspensionWorldDirection * castDistance,
                            tmpVehicleWheelData.WheelWorldTransform.rot);
                        WheelClosestHitCollector collector = new WheelClosestHitCollector(entity);

                        if (PhysicsWorld.CastCollider(colliderCastInput, ref collector))
                        {
                            if (math.dot(collector.ClosestHit.SurfaceNormal, tmpVehicleWheelData.WheelUp) > 0f)
                            {
                                // There is a hit, store it.
                                wheelOnVehicle.WheelHit = collector.ClosestHit;
                                wheelOnVehicle.IsGrounded = true;

                                wheelOnVehicle.SuspensionLength = math.clamp(
                                    wheelOnVehicle.WheelHit.Fraction * castDistance, 0f, wheel.SuspensionMaxLength);

                                VehicleUtilities.GetHitBodyVelocity(in PhysicsWorld,
                                    in wheelOnVehicle.WheelHit,
                                    out tmpVehicleWheelData.HitRigidbodyVelocity);
                            }
                        }
                        else
                        {
                            // No hit, wheel is in the air. Extend it fully instantly to prevent jitter in case of moving platforms, etc.
                            wheelOnVehicle.SuspensionLength = wheel.SuspensionMaxLength;
                        }
                    }

                    // Brake torque can only be positive
                    wheelOnVehicle.BrakeTorque = math.clamp(wheelOnVehicle.BrakeTorque, 0f, math.INFINITY);

                    // Cut the motor torque to 0 if brakes are applied
                    if (wheelOnVehicle.BrakeTorque != 0)
                    {
                        wheelOnVehicle.MotorTorque = 0f;
                    }

                    // Update wheel transforms and directions after calculating the suspension length
                    tmpVehicleWheelData.UpdateWheelTransforms(in wheelOnVehicle);

                    tmpVehicleWheelData.SubsteppedSuspensionLength = wheelOnVehicle.SuspensionLength;

                    vehicleWheelsBuffer[wheelIndex] = wheelOnVehicle;
                    _tmpVehicleWheelDatas[wheelIndex] = tmpVehicleWheelData;
                }

                // Sub-step friction and suspension
                float stepDeltaTime = vehicle.SubstepCount == 0
                    ? VehicleTimeSingleton.LastPhysicsUpdateDeltaTime
                    : VehicleTimeSingleton.LastPhysicsUpdateDeltaTime / vehicle.SubstepCount;
                float invStepDeltaTime = stepDeltaTime == 0f ? 0f : 1f / stepDeltaTime;

                float2 frictionMagnitude = default;
                float2 maxFrictionForce = default;
                float2 absSpeed = default;
                float3 gravityVelocity = Gravity * stepDeltaTime;

                // Pre-calculate values that do not change
                int lngFrictionSlices = 12;
                float invLngFrictionSlices = 1f / (float)lngFrictionSlices;
                float lngSliceDeltaTime = stepDeltaTime / (float)lngFrictionSlices;

                // Substep start
                for (int substep = 0; substep < vehicle.SubstepCount; substep++)
                {
                    for (int wheelIndex = 0; wheelIndex < vehicleWheelsBuffer.Length; wheelIndex++)
                    {
                        WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[wheelIndex];
                        TmpVehicleWheelData tmpVehicleWheelData = _tmpVehicleWheelDatas[wheelIndex];
                        Wheel wheel = wheelOnVehicle.Wheel;

                        float suspensionLoad = 0f;

                        // Velocity and speeds

                        // Get vehicle velocity at the hit point
                        float3 frictionPointVelocity = physicsVelocity.GetLinearVelocity(physicsMass,
                            transform.Position, transform.Rotation, tmpVehicleWheelData.CenteredThreadPoint);

                        // Subtract hit Rigidbody velocity, if any
                        frictionPointVelocity -= tmpVehicleWheelData.HitRigidbodyVelocity;

                        // Get individual speed components
                        float3 projectedVelocity = MathUtilities.ProjectOnPlane(frictionPointVelocity,
                            wheelOnVehicle.WheelHit.SurfaceNormal);
                        wheelOnVehicle.FrictionSpeed.x = math.dot(projectedVelocity, tmpVehicleWheelData.WheelForward);
                        wheelOnVehicle.FrictionSpeed.y = math.dot(projectedVelocity, tmpVehicleWheelData.WheelRight);
                        absSpeed.x = math.abs(wheelOnVehicle.FrictionSpeed.x);
                        absSpeed.y = math.abs(wheelOnVehicle.FrictionSpeed.y);

                        // ===== Suspension =====
                        if (wheelOnVehicle.IsGrounded)
                        {
                            float3 suspensionPointVelocity = physicsVelocity.GetLinearVelocity(physicsMass,
                                                                 transform.Position, transform.Rotation,
                                                                 tmpVehicleWheelData.SteeredSuspensionWorldTransform
                                                                     .pos)
                                                             + gravityVelocity;

                            suspensionPointVelocity -= tmpVehicleWheelData.HitRigidbodyVelocity;

                            float prevSubsteppedSuspensionLength = tmpVehicleWheelData.SubsteppedSuspensionLength;

                            tmpVehicleWheelData.SubsteppedSuspensionLength +=
                                math.dot(suspensionPointVelocity, wheelOnVehicle.WheelHit.SurfaceNormal) *
                                stepDeltaTime;
                            tmpVehicleWheelData.SubsteppedSuspensionLength = math.clamp(
                                tmpVehicleWheelData.SubsteppedSuspensionLength, 0f,
                                wheel.SuspensionMaxLength);

                            // Spring
                            float springRatePerMeter =
                                wheel.SuspensionSpringRate * 1000f; // Standard unit N/mm, so convert to N/m.
                            float suspensionImpulseMagnitude =
                                (wheel.SuspensionMaxLength - tmpVehicleWheelData.SubsteppedSuspensionLength) *
                                springRatePerMeter;

                            //Damper 
                            float suspensionCompressionSpeed =
                                (tmpVehicleWheelData.SubsteppedSuspensionLength - prevSubsteppedSuspensionLength) *
                                invStepDeltaTime;
                            suspensionImpulseMagnitude -= suspensionCompressionSpeed * wheel.SuspensionDampingRate;

                            suspensionImpulseMagnitude = math.clamp(suspensionImpulseMagnitude, 0f, math.INFINITY);

                            tmpVehicleWheelData.SuspensionStepImpulse = suspensionImpulseMagnitude
                                                                        * stepDeltaTime
                                                                        * wheelOnVehicle.WheelHit.SurfaceNormal;
                            wheelOnVehicle.SuspensionImpulse += tmpVehicleWheelData.SuspensionStepImpulse;

                            // Get wheel load.
                            // Suspension is not taking into account the wheel mass so the load is equal to the suspension force.
                            // Clamped because negative tire load is not a thing as it can not attract the surface to itself, only push.
                            suspensionLoad = math.clamp(suspensionImpulseMagnitude, 0f, math.INFINITY);
                        }
                        else
                        {
                            wheelOnVehicle.SuspensionImpulse = 0f;
                            tmpVehicleWheelData.SuspensionStepImpulse = 0f;
                        }

                        // Calculate load with no suspension
                        if (wheel.SuspensionMaxLength <= 0f)
                        {
                            if (wheelOnVehicle.IsGrounded)
                            {
                                float verticalEffectiveMass = physicsMass.GetEffectiveMass(in physicsMass.Transform.pos,
                                    in physicsMass.Transform.rot, tmpVehicleWheelData.SteeredSuspensionWorldDirection,
                                    tmpVehicleWheelData.CenteredThreadPoint);
                                suspensionLoad = verticalEffectiveMass *
                                                 math.dot(Gravity, tmpVehicleWheelData.SteeredSuspensionWorldDirection);

                            }
                        }

                        // ===== Friction =====

                        // Calculate maximum friction force that can be applied overall, used to clamp friction when combining 
                        // different friction sources such as the anti-creep and speed limiter
                        // If the slip is below the minimum, assume that the maximum can be achieved during the substep, this is
                        // to avoid the complete loss of friction during the first step where the slip might be 0.
                        float absSlipX = math.abs(wheelOnVehicle.FrictionSlip.x);
                        maxFrictionForce.x = absSlipX < wheel.FrictionCurveMaximum.x
                            ? wheel.FrictionCurveMaximum.y * suspensionLoad
                            : EvaluateFrictionCurve(absSlipX, in wheel.FrictionCurveMaximum,
                                in wheel.FrictionCurveMinimum) * suspensionLoad;

                        float absSlipY = math.abs(wheelOnVehicle.FrictionSlip.y);
                        maxFrictionForce.y = absSlipY < wheel.FrictionCurveMaximum.x
                            ? wheel.FrictionCurveMaximum.y * suspensionLoad
                            : EvaluateFrictionCurve(absSlipY, in wheel.FrictionCurveMaximum,
                                in wheel.FrictionCurveMinimum) * suspensionLoad;

                        // ===== Lateral friction =====

                        // Calculate slip in degrees and scale it to a normalized friction curve (0 to 1 representing 0 to 90 degrees).
                        float clampedLongitudinalSpeed =
                            absSpeed.x < vehicle.VirtualScale ? vehicle.VirtualScale : absSpeed.x;
                        wheelOnVehicle.FrictionSlip.y = wheelOnVehicle.IsGrounded
                            ? math.atan2(wheelOnVehicle.FrictionSpeed.y, clampedLongitudinalSpeed) * math.TODEGREES *
                              0.01111f
                            : 0f;
                        frictionMagnitude.y = (wheelOnVehicle.FrictionSlip.y >= 0f ? -1f : 1f) *
                                              EvaluateFrictionCurve(math.abs(wheelOnVehicle.FrictionSlip.y),
                                                  in wheel.FrictionCurveMaximum,
                                                  in wheel.FrictionCurveMinimum) * suspensionLoad;


                        // ===== Longitudinal friction =====

                        // Longitudinal slip
                        // Longitudinal friction is extremely stiff, so slice this into even smaller steps.
                        // This is done in attempt to prevent the slip from going from x = 0 to x = 1 (maximum slip 
                        // represented on the friction graph) during one physics update.
                        float invInertiaTimesLngDeltaTime = tmpVehicleWheelData.InvMomentOfInertia * lngSliceDeltaTime;
                        float motorTorqueAngVelIncrement = wheelOnVehicle.MotorTorque * invInertiaTimesLngDeltaTime;
                        float brakeTorqueAngVelIncrement = wheelOnVehicle.BrakeTorque * invInertiaTimesLngDeltaTime;
                        float c = wheel.Radius * invInertiaTimesLngDeltaTime;
                        float clampedSpeedX = math.clamp(math.abs(wheelOnVehicle.FrictionSpeed.x), 0.2f, math.INFINITY)
                                              * math.sign(wheelOnVehicle.FrictionSpeed.x);

                        // During these slices the input torques stay constant, but the value of the friction graph
                        // will change depending on the slip, effectively integrating the friction curve with higher resolution
                        // than might be possible with the selected number of sub-steps.
                        for (int i = 0; i < lngFrictionSlices; i++)
                        {
                            // Apply motor torque
                            if (wheelOnVehicle.BrakeTorque == 0f)
                            {
                                wheelOnVehicle.AngularVelocity += motorTorqueAngVelIncrement;
                            }

                            // Calculate slip and friction
                            if (wheelOnVehicle.IsGrounded)
                            {
                                // >0 = wheel spin, <0 = wheel slip
                                float speedDelta = wheelOnVehicle.AngularVelocity * wheel.Radius -
                                                   wheelOnVehicle.FrictionSpeed.x;
                                wheelOnVehicle.FrictionSlip.x = speedDelta / clampedSpeedX;
                                float sliceFrictionForce =
                                    math.sign(speedDelta)
                                    * suspensionLoad
                                    * EvaluateFrictionCurve(
                                        math.abs(wheelOnVehicle.FrictionSlip.x),
                                        in wheel.FrictionCurveMaximum,
                                        in wheel.FrictionCurveMinimum);
                                wheelOnVehicle.AngularVelocity -= sliceFrictionForce * c;
                                frictionMagnitude.x += sliceFrictionForce * invLngFrictionSlices;
                            }

                            // Apply brake torque
                            if (wheelOnVehicle.BrakeTorque > 0f)
                            {
                                float preBrakeAngVel = wheelOnVehicle.AngularVelocity;

                                // Do this after the friction is applied to be able to fully lock the wheel.
                                wheelOnVehicle.AngularVelocity -= wheelOnVehicle.AngularVelocity > 0
                                    ? brakeTorqueAngVelIncrement
                                    : -brakeTorqueAngVelIncrement;

                                // Do not allow zero crossing during one step to prevent brakes from aceelerating the wheel 
                                // in the opposite direction, as brakes should only reduce the angular velocity.
                                //if (MathUtilities.Sign(preBrakeAngVel) != MathUtilities.Sign(vehicleWheel.AngularVelocity))
                                if (preBrakeAngVel < 0f && wheelOnVehicle.AngularVelocity >= 0f ||
                                    preBrakeAngVel > 0f && wheelOnVehicle.AngularVelocity <= 0f)
                                {
                                    wheelOnVehicle.AngularVelocity = 0f;
                                }
                            }
                        }

                        // Update the longitudinal slip one last time, after the brakes are applied
                        wheelOnVehicle.FrictionSlip.x = wheelOnVehicle.IsGrounded
                            ? (wheelOnVehicle.AngularVelocity * wheel.Radius - wheelOnVehicle.FrictionSpeed.x) /
                              clampedSpeedX
                            : 0f;

                        // Clamp the output friction force to the input force
                        if (wheelOnVehicle.BrakeTorque > 0f)
                        {
                            float inputBrakeForce = wheelOnVehicle.BrakeTorque * tmpVehicleWheelData.InvRadius;
                            frictionMagnitude.x = math.clamp(frictionMagnitude.x, -inputBrakeForce, inputBrakeForce);
                        }
                        else
                        {
                            float inputMotorForce =
                                math.abs(wheelOnVehicle.MotorTorque) * tmpVehicleWheelData.InvRadius;
                            frictionMagnitude.x = math.clamp(frictionMagnitude.x, -inputMotorForce, inputMotorForce);
                        }

                        // Normalize the slip values.
                        // Values being in -1 to 1 range are more useful for effects, sounds, etc. and the 
                        // slip outside the [-1, 1] range is also not represented in the fricton curve.
                        wheelOnVehicle.FrictionSlip.x = math.clamp(wheelOnVehicle.FrictionSlip.x, -1f, 1f);
                        wheelOnVehicle.FrictionSlip.y = math.clamp(wheelOnVehicle.FrictionSlip.y, -1f, 1f);


                        // ===== Speed limiting =====
                        if (wheelOnVehicle.AngularVelocityLimit != 0f)
                        {
                            wheelOnVehicle.AngularVelocity = math.clamp(wheelOnVehicle.AngularVelocity,
                                -wheelOnVehicle.AngularVelocityLimit, wheelOnVehicle.AngularVelocityLimit);

                            if (wheelOnVehicle.IsGrounded)
                            {
                                float speedLimit = wheelOnVehicle.AngularVelocityLimit * wheel.Radius;
                                float overspeedError = absSpeed.x - speedLimit;
                                if (overspeedError > 0f)
                                {
                                    float speedCorrectionImpulseMagnitude = suspensionLoad * overspeedError *
                                                                            (wheelOnVehicle.FrictionSpeed.x >= 0f
                                                                                ? -1f
                                                                                : 1f);
                                    frictionMagnitude.x += speedCorrectionImpulseMagnitude;
                                }
                            }
                        }

                        // ===== Static friction =====
                        // Used to prevent creep on slopes at near 0 velocity due to the slip based nature of the 
                        // main friction calculation. 
                        float staticFrictionThresholdVelocity = vehicle.VirtualScale * 0.2f;
                        if (!wheelOnVehicle.DisableStaticFrictionSingleFrame
                            && wheelOnVehicle.IsGrounded
                            && absSpeed.x < staticFrictionThresholdVelocity
                            && absSpeed.y < staticFrictionThresholdVelocity
                            && wheelOnVehicle.MotorTorque == 0f
                            && math.abs(wheelOnVehicle.AngularVelocity) < 0.5f)
                        {
                            if (!wheelOnVehicle.StaticFrictionReferenceIsSet)
                            {
                                wheelOnVehicle.StaticFrictionReferenceIsSet = true;
                                wheelOnVehicle.StaticFrictionRefPosition = tmpVehicleWheelData.CenteredThreadPoint;
                            }
                            else
                            {
                                // Calculate offset from the reference point, but only on the horizontal plane. 
                                // The suspension direction is taken here as a plane normal as the hit normal might suddenly change if e.g.
                                // a step is encountered with the small amount of drift that is still possible using this method.
                                wheelOnVehicle.StaticFrictionRefPosition +=
                                    MathUtilities.ProjectOnPlane(
                                        tmpVehicleWheelData.HitRigidbodyVelocity * stepDeltaTime,
                                        tmpVehicleWheelData.WheelUp);
                                float3 offsetFromReference = MathUtilities.ProjectOnPlane(
                                    wheelOnVehicle.StaticFrictionRefPosition - tmpVehicleWheelData.CenteredThreadPoint,
                                    tmpVehicleWheelData.WheelUp);
                                float3 positionImpulse = offsetFromReference * 1e4f * suspensionLoad * stepDeltaTime;
                                frictionMagnitude.x += math.dot(positionImpulse, tmpVehicleWheelData.WheelForward);
                                frictionMagnitude.y += math.dot(positionImpulse, tmpVehicleWheelData.WheelRight);
                            }
                        }
                        else
                        {
                            wheelOnVehicle.StaticFrictionReferenceIsSet = false;
                        }

                        wheelOnVehicle.DisableStaticFrictionSingleFrame = false;

                        // Clamp the magnitudes of individual friction components after all the additional forces are added
                        // to make sure the final force is not exceeding what the friction can provide.
                        frictionMagnitude.x = math.clamp(frictionMagnitude.x, -maxFrictionForce.x, maxFrictionForce.x);
                        frictionMagnitude.y = math.clamp(frictionMagnitude.y, -maxFrictionForce.y, maxFrictionForce.y);

                        // ===== Friction circle =====

                        // Lateral friction is reduced with the wheel spin.
                        float longitudinalSlipRatio =
                            math.saturate(math.abs(wheelOnVehicle.FrictionSlip.x) / wheel.FrictionCurveMaximum.x)
                            * wheel.FrictionCircleStrength;
                        frictionMagnitude.y *= 1f - math.pow(longitudinalSlipRatio, 3f);

                        // Calculate combined friction impulse
                        tmpVehicleWheelData.FrictionStepImpulse =
                            (frictionMagnitude.x * tmpVehicleWheelData.WheelForward
                             + frictionMagnitude.y * tmpVehicleWheelData.WheelRight) * stepDeltaTime;
                        wheelOnVehicle.FrictionImpulse += tmpVehicleWheelData.FrictionStepImpulse;

                        // Assign back the new values
                        vehicleWheelsBuffer[wheelIndex] = wheelOnVehicle;
                        _tmpVehicleWheelDatas[wheelIndex] = tmpVehicleWheelData;
                    }

                    // Calculate impulses
                    float3 totalLinearImpulse = float3.zero;
                    float3 totalAngularImpulse = float3.zero;
                    float3 worldCoM = transform.TransformPoint(physicsMass.CenterOfMass);

                    for (int wheelIndex = 0; wheelIndex < vehicleWheelsBuffer.Length; wheelIndex++)
                    {
                        WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[wheelIndex];
                        TmpVehicleWheelData tmpVehicleWheelData = _tmpVehicleWheelDatas[wheelIndex];
                        Wheel wheel = wheelOnVehicle.Wheel;

                        // ARB
                        if (wheelOnVehicle.AxlePairedWheelIndex >= 0
                            && wheel.AntiRollBarStiffness > 0f
                            && wheel.SuspensionMaxLength > 0f)
                        {
                            if (wheelOnVehicle.AxlePairedWheelIndex >= 0)
                            {
                                WheelOnVehicle wheelOnVehicleSibling =
                                    vehicleWheelsBuffer[wheelOnVehicle.AxlePairedWheelIndex];
                                TmpVehicleWheelData tmpVehicleWheelDataSibling =
                                    _tmpVehicleWheelDatas[wheelOnVehicle.AxlePairedWheelIndex];

                                // There is no feedback between the spring length and the ARB, so make sure ARB is only applied if both
                                // wheels are on the ground, or one of the wheels suddenly extending will generate unwanted large forces on the paired wheel.
                                if (wheelOnVehicle.IsGrounded && wheelOnVehicleSibling.IsGrounded)
                                {
                                    float springLengthDifference =
                                        tmpVehicleWheelDataSibling.SubsteppedSuspensionLength -
                                        tmpVehicleWheelData.SubsteppedSuspensionLength;

                                    // Add ARB force to the suspension impulse
                                    float arbStepImpulse = springLengthDifference * wheel.AntiRollBarStiffness
                                        * 0.5f * stepDeltaTime;
                                    tmpVehicleWheelData.SuspensionStepImpulse += arbStepImpulse;
                                }
                            }
                        }

                        // Add suspension impulse
                        totalLinearImpulse += tmpVehicleWheelData.SuspensionStepImpulse;
                        totalAngularImpulse += math.cross(
                            tmpVehicleWheelData.SteeredSuspensionWorldTransform.pos - worldCoM,
                            tmpVehicleWheelData.SuspensionStepImpulse);
                        // UnityEngine.Debug.DrawRay(vehicleTransform.pos, tmpVehicleWheelData.SuspensionStepImpulse,
                        //     Color.yellow, 10f);

                        // Add friction impulse
                        totalLinearImpulse += tmpVehicleWheelData.FrictionStepImpulse;
                        totalAngularImpulse += math.cross(tmpVehicleWheelData.CenteredThreadPoint - worldCoM,
                            tmpVehicleWheelData.FrictionStepImpulse);
                        // UnityEngine.Debug.DrawRay(vehicleTransform.pos, tmpVehicleWheelData.FrictionStepImpulse,
                        //     Color.blue, 10f);
                    }

                    // Apply linear and angular impulse
                    physicsVelocity.ApplyLinearImpulse(physicsMass, totalLinearImpulse);

                    float3 localTotalAngularImpulse = transform.InverseTransformDirection(totalAngularImpulse);
                    physicsVelocity.ApplyAngularImpulse(physicsMass, localTotalAngularImpulse);
                }
                // Substep end

                // Final wheels pass for transforms, spin, colliders and impulses
                for (int wheelIndex = 0; wheelIndex < vehicleWheelsBuffer.Length; wheelIndex++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[wheelIndex];

                    // Update wheel spin rotation
                    {
                        wheelOnVehicle.RotationAngle += wheelOnVehicle.AngularVelocity *
                                                        VehicleTimeSingleton.LastPhysicsUpdateDeltaTime;
                        // Modulo by 2PI radians to keep in the range one full rotation
                        wheelOnVehicle.RotationAngle %= math.PI2;
                    }

                    // Apply impulses to wheel hits
                    DeferredImpulses.Write(new DeferredImpulse
                    {
                        Impulse = -wheelOnVehicle.SuspensionImpulse - wheelOnVehicle.FrictionImpulse,
                        Position = wheelOnVehicle.WheelHit.Position,
                        Entity = wheelOnVehicle.WheelHit.Entity,
                    });

                    vehicleWheelsBuffer[wheelIndex] = wheelOnVehicle;
                }
            }

            /// <summary>
            /// On chunk begin
            /// </summary>
            /// <param name="chunk">The chunk</param>
            /// <param name="unfilteredChunkIndex">The unfiltered chunk index</param>
            /// <param name="useEnabledMask">If using enabled mask</param>
            /// <param name="chunkEnabledMask">The chunk enabled mask</param>
            /// <returns>True if chunk should be executed.</returns>
            public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask)
            {
                DeferredImpulses.BeginForEachIndex(unfilteredChunkIndex);

                if (!_tmpVehicleWheelDatas.IsCreated)
                {
                    _tmpVehicleWheelDatas = new NativeList<TmpVehicleWheelData>(16, Allocator.Temp);
                }

                return true;
            }

            /// <summary>
            /// On chunk end
            /// </summary>
            /// <param name="chunk">The chunk</param>
            /// <param name="unfilteredChunkIndex">The unfiltered chunk index</param>
            /// <param name="useEnabledMask">If using enabled mask</param>
            /// <param name="chunkEnabledMask">The chunk enabled mask</param>
            /// <param name="chunkWasExecuted">If the chunk was executed</param>
            public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask,
                bool chunkWasExecuted)
            {
                DeferredImpulses.EndForEachIndex();
            }
        }

        /// <summary>
        /// Get the friction curve value y at time t.
        /// </summary>
        /// <param name="x">X-axis value at which to evaluate.</param>
        /// <param name="curveMaximum">Maximum/peak of the curve.</param>
        /// <param name="curveMinimum">Minimum of the curve, after the peak.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float EvaluateFrictionCurve(float t, in float2 curveMaximum, in float2 curveMinimum)
        {
            float y = 0;

            Assert.IsTrue(curveMaximum.x > 0, "Curve maximum x value should be >0.");
            Assert.IsTrue(curveMaximum.x < curveMinimum.x,
                "Curve maximum should have a lower x value than curve minimum.");

            if (t <= 0 || curveMaximum.x <= 0 || curveMinimum.x <= 0)
            {
                return 0;
            }
            else if (t > curveMinimum.x)
            {
                return curveMinimum.y;
            }
            else if (t > 0 && t <= curveMaximum.x)
            {
                return math.lerp(0, curveMaximum.y, (t / curveMaximum.x));
            }
            else if (t > curveMaximum.x && t <= curveMinimum.x)
            {
                return math.lerp(curveMaximum.y, curveMinimum.y,
                    (t - curveMaximum.x) / (curveMinimum.x - curveMaximum.x));
            }

            return y;
        }

        /// <summary>
        /// Applies impulses that were cached in the <see cref="SolveVehiclePhysicsJob"/>
        /// </summary>
        [BurstCompile]
        public struct ApplyDeferredImpulsesJob : IJob
        {
            /// <summary>
            /// Physics velocity component lookup
            /// </summary>
            public ComponentLookup<PhysicsVelocity> PhysicsVelocityLookup;
            /// <summary>
            /// Physics mass component lookup
            /// </summary>
            [ReadOnly] public ComponentLookup<PhysicsMass> PhysicsMassLookup;
            /// <summary>
            /// Local transform component lookup
            /// </summary>
            [ReadOnly] public ComponentLookup<LocalTransform> LocalTransformLookup;
            /// <summary>
            /// Deferred impulses stream
            /// </summary>
            public NativeStream.Reader DeferredImpulses;

            /// <summary>
            /// The execute function
            /// </summary>
            public void Execute()
            {
                for (int i = 0; i < DeferredImpulses.ForEachCount; i++)
                {
                    DeferredImpulses.BeginForEachIndex(i);
                    while (DeferredImpulses.RemainingItemCount > 0)
                    {
                        DeferredImpulse impulse = DeferredImpulses.Read<DeferredImpulse>();
                        if (PhysicsVelocityLookup.TryGetComponent(impulse.Entity,
                                out PhysicsVelocity physicsVelocity) &&
                            PhysicsMassLookup.TryGetComponent(impulse.Entity, out PhysicsMass physicsMass) &&
                            LocalTransformLookup.TryGetComponent(impulse.Entity, out LocalTransform localTransform))
                        {
                            if (!physicsMass.IsKinematic)
                            {
                                physicsVelocity.ApplyImpulse(in physicsMass, localTransform.Position,
                                    localTransform.Rotation, localTransform.Scale, impulse.Impulse, impulse.Position);
                                PhysicsVelocityLookup[impulse.Entity] = physicsVelocity;
                            }
                        }
                    }

                    DeferredImpulses.EndForEachIndex();
                }
            }
        }
    }
}