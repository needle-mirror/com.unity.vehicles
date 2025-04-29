using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using Unity.Physics.Extensions;
using UnityEngine;

namespace Unity.Vehicles
{
    /// <summary>
    /// System that handles control of the vehicle, including powertrain, steering and aerodynamics.
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
    [UpdateBefore(typeof(FixedStepSimulationSystemGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    public partial struct VehicleControlSystem : ISystem
    {
        internal void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsWorldSingleton>();
        }
        
        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            VehicleControlJob job = new VehicleControlJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                PhysicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld,
                WheelControlLookup = SystemAPI.GetComponentLookup<WheelControl>(true),
                EngineControlLookup = SystemAPI.GetComponentLookup<EngineStartStop>(false),
            };
            state.Dependency = job.ScheduleParallel(state.Dependency);

            state.Dependency = new VehicleEngineControlJob
            {
                EngineControlLookup = SystemAPI.GetComponentLookup<EngineStartStop>(false),
            }.Schedule(state.Dependency);
        }

        /// <summary>
        /// Updates the powertrain and sends the torque to the <see cref="WheelControl"/>.
        /// Also handles steering and aerodynamics.
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct VehicleControlJob : IJobEntity
        {
            /// <summary>
            /// Time since the last update.
            /// </summary>
            public float DeltaTime;

            /// <summary>
            /// The physics world
            /// </summary>
            [ReadOnly] public PhysicsWorld PhysicsWorld;
            
            /// <summary>
            /// <see cref="WheelControl"/> lookup.
            /// </summary>
            [ReadOnly] 
            public ComponentLookup<WheelControl> WheelControlLookup;

            /// <summary>
            /// <see cref="EngineStartStop"/> lookup.
            /// </summary>
            [NativeDisableParallelForRestriction] 
            public ComponentLookup<EngineStartStop> EngineControlLookup;

            void Execute(
                Entity entity,
                in LocalTransform transform,
                in PhysicsMass physicsMass,
                ref DynamicBuffer<WheelOnVehicle> vehicleWheels,
                ref PhysicsVelocity physicsVelocity,
                ref Vehicle vehicle,
                ref VehicleControl vehicleControl,
                ref VehicleControlData vehicleControlData,
                ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                // Clear events
                controlEvents.Clear();

                DeltaTime = DeltaTime < 1e-6f ? 1e-6f : DeltaTime;

                // Get vehicle forward speed
                float3 localVelocity = transform.InverseTransformDirection(physicsVelocity.Linear);
                float vehicleForwardSpeed = localVelocity.z;

                // Update steering
                {
                    // Update steering position
                    float targetSteerPosition = math.lerp(
                        vehicleControlData.SteeringSensitivityLowSpeed * vehicleControl.RawSteeringInput,
                        vehicleControlData.SteeringSensitivityHighSpeed * vehicleControl.RawSteeringInput,
                        math.abs(vehicleForwardSpeed) / vehicleControlData.SteeringSensitivitySpeedReference);

                    // Use a step instead of lerp to make sure that the steering value change is linear, even 
                    // when approaching the target value.
                    vehicleControlData.SteeringPosition = MathUtilities.StepTowards(vehicleControlData.SteeringPosition,
                        targetSteerPosition, vehicleControlData.SteerSpeed * DeltaTime);
                }
                
                // Get the angular velocity from the wheels
                int drivenWheelCount = 0;
                float drivenWheelMotorTorqueCoefficientSum = 0f;
                float referenceAngularVelocity = vehicleControlData.EngineAngularVelocityCalculation == VehicleControlAuthoring.EngineAngularVelocityCalculationType.Minimum ? math.INFINITY : 0f;
                float referenceWheelRadius = 0f;
                float3 referenceHitRigidbodyVelocity = float3.zero;
                for (int wheelIndex = 0; wheelIndex < vehicleWheels.Length; wheelIndex++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheels[wheelIndex];
                    if (WheelControlLookup.TryGetComponent(wheelOnVehicle.Entity, out WheelControl wheelControl) &&
                        wheelControl.MotorTorqueCoefficient > 0f)
                    {
                        Wheel wheel = wheelOnVehicle.Wheel;

                        drivenWheelCount++;
                        drivenWheelMotorTorqueCoefficientSum += wheelControl.MotorTorqueCoefficient;

                        // Maximum
                        if (vehicleControlData.EngineAngularVelocityCalculation == VehicleControlAuthoring.EngineAngularVelocityCalculationType.Maximum)
                        {
                            if (math.abs(wheelOnVehicle.AngularVelocity) > referenceAngularVelocity)
                            {
                                referenceAngularVelocity = wheelOnVehicle.AngularVelocity;
                                referenceWheelRadius = wheel.Radius;
                                VehicleUtilities.GetHitBodyVelocity(in PhysicsWorld, in wheelOnVehicle.WheelHit, out referenceHitRigidbodyVelocity);
                            }
                        }
                        // Minimum
                        else if (vehicleControlData.EngineAngularVelocityCalculation == VehicleControlAuthoring.EngineAngularVelocityCalculationType.Minimum)
                        {
                            if (math.abs(wheelOnVehicle.AngularVelocity) < referenceAngularVelocity)
                            {
                                referenceAngularVelocity = wheelOnVehicle.AngularVelocity;
                                referenceWheelRadius = wheel.Radius;
                                VehicleUtilities.GetHitBodyVelocity(in PhysicsWorld, in wheelOnVehicle.WheelHit, out referenceHitRigidbodyVelocity);
                            }
                        }
                        // Average
                        else
                        {
                            referenceAngularVelocity += wheelOnVehicle.AngularVelocity;
                            referenceWheelRadius += wheel.Radius;
                            VehicleUtilities.GetHitBodyVelocity(in PhysicsWorld, in wheelOnVehicle.WheelHit, out referenceHitRigidbodyVelocity);
                        }
                    }
                }

                // Calculate average
                if (vehicleControlData.EngineAngularVelocityCalculation == VehicleControlAuthoring.EngineAngularVelocityCalculationType.Average
                    && drivenWheelCount > 0)
                {
                    referenceAngularVelocity /= drivenWheelCount;
                    referenceWheelRadius /= drivenWheelCount;
                    referenceHitRigidbodyVelocity /= drivenWheelCount;
                }
                
                // Calculate wheel linear speed
                vehicleControlData.WheelSpeed = referenceAngularVelocity * referenceWheelRadius;

                // Calculate relative speed of the vehicle taking into the account moving Rigidbodies the vehicle might be on or touching.
                float hitRigidbodyForwardSpeed = math.dot(referenceHitRigidbodyVelocity, transform.Forward());
                vehicleControlData.RelativeForwardSpeed = vehicleForwardSpeed - hitRigidbodyForwardSpeed;

                // Simplistic clutch to disable the connection between the wheels and the engine.
                bool isEngineConnectedToWheels = vehicleControlData.TransmissionCurrentGearRatio != 0f && drivenWheelCount > 0;
                
                // Process throttle and brake inputs.
                {
                    float swapInputSpeedThreshold = 0.3f * vehicle.VirtualScale;

                    // Flip the input depending on vehicle travel direction
                    if (vehicleControlData.SwapThrottleAndBrakeInReverse
                        && (vehicleControlData.RelativeForwardSpeed < -swapInputSpeedThreshold ||
                            vehicleControlData.TransmissionCurrentGearRatio < 0f)
                        && vehicleControlData.WheelSpeed < swapInputSpeedThreshold)
                    {
                        // We are in reverse and throttle and brake swap is on, use brake to accelerate and throttle to slow down.
                        vehicleControlData.ThrottlePosition = vehicleControl.RawBrakeInput;
                        vehicleControlData.BrakePosition = vehicleControl.RawThrottleInput;
                    }
                    else
                    {
                        // Normal throttle and brake input, throttle always accelerates and brake always slows the vehicle down.
                        vehicleControlData.ThrottlePosition = vehicleControl.RawThrottleInput;
                        vehicleControlData.BrakePosition = vehicleControl.RawBrakeInput;
                    }
                }
                
                // Engine
                {
                    // Start/stop the engine through user input
                    if (vehicleControl.EngineStartStopInput)
                    {
                        if (vehicleControlData.EngineIsRunning)
                        {
                            StopEngine(entity);
                        }
                        else
                        {
                            StartEngine(entity);
                        }
                    }

                    // Start engine through throttle
                    if (!vehicleControlData.EngineIsRunning && vehicleControlData.EngineStartOnThrottle &&
                        vehicleControlData.ThrottlePosition > 0f)
                    {
                        StartEngine(entity);
                    }

                    // Update engine angular velocity from wheel angular velocities, if connected to the wheels.
                    if (isEngineConnectedToWheels)
                    {
                        vehicleControlData.EngineAngularVelocity =
                            referenceAngularVelocity * vehicleControlData.TransmissionCurrentGearRatio;
                    }

                    // Calculate engine torque from RPM and throttle position
                    float engineAngularVelocityPercent = vehicleControlData.EngineMaxAngularVelocity == 0f
                        ? 0f
                        : math.saturate(vehicleControlData.EngineAngularVelocity / vehicleControlData.EngineMaxAngularVelocity);

                    float engineTorqueCoefficient;
                    if (vehicleControlData.EngineIsRunning && vehicleControlData.ThrottlePosition > 0f)
                    {
                        engineTorqueCoefficient = vehicleControlData.ThrottlePosition;
                    }
                    else
                    {
                        // Engine is not running or there is no throttle input, apply engine braking
                        engineTorqueCoefficient = -vehicleControlData.EngineBrakingIntensity;
                    }

                    vehicleControlData.EngineTorque =
                        vehicleControlData.EngineTorqueCurve.Value.EvaluateAt(engineAngularVelocityPercent)
                        * vehicleControlData.EngineMaxTorque * engineTorqueCoefficient;

                    // Engine is not connected to the wheels, spin it up like a flywheel
                    if (!isEngineConnectedToWheels)
                    {
                        vehicleControlData.EngineAngularVelocity +=
                            (vehicleControlData.EngineTorque / vehicleControlData.EngineAngularInertia) * DeltaTime;
                    }

                    // Clamp the engine angular velocity to the rev limiter range.
                    // Do not allow negative angular velocity as this is what reverse gears are for.
                    vehicleControlData.EngineAngularVelocity = math.clamp(vehicleControlData.EngineAngularVelocity, 0f,
                        vehicleControlData.EngineMaxAngularVelocity);
                }
                
                // Transmission
                float transmissionTorque;
                {
                    // Shift gears as needed.
                    if (vehicleControlData.TransmissionIsAutomatic)
                    {
                        // Passing in the actual wheel speed here would be more accurate but causes issues with upshifting in air,
                        // downshifting when wheels are locked up, etc.
                        AutoShift(in vehicleControlData.RelativeForwardSpeed, in referenceWheelRadius, in vehicleControl, ref vehicleControlData,
                            ref vehicle, ref controlEvents);
                    }
                    else
                    {
                        ManualShift(in vehicleControl, ref vehicleControlData, ref controlEvents);
                    }

                    // Calculate the transmission output torque from the engine torque and the total gear ratio.
                    transmissionTorque =
                        vehicleControlData.TransmissionCurrentGearRatio * vehicleControlData.EngineTorque;
                }

                // Update wheels
                for (int wheelIndex = 0; wheelIndex < vehicleWheels.Length; wheelIndex++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheels[wheelIndex];
                    if (WheelControlLookup.TryGetComponent(wheelOnVehicle.Entity, out WheelControl wheelControl))
                    {
                        Wheel wheel = wheelOnVehicle.Wheel;

                        // Assign steering angle
                        if (wheelControl.MaxSteerAngle != 0)
                        {
                            wheelOnVehicle.SteerAngle = vehicleControlData.SteeringPosition * wheelControl.MaxSteerAngle;
                            
                            // Add Ackerman steering
                            if (wheelControl.AckermanCoefficient != 0)
                            {
                                // If there is a wheel sibling, and this wheel is the outside one, apply Ackerman steering to this wheel
                                float vehicleSide = wheelOnVehicle.SuspensionLocalTransform.pos.x < 0f ? -1f : 1f;

                                // If the vehicle side of the wheel is the same as the direction of steering,
                                // increase the steer angle on the wheel as it is on the inside of the steering circle
                                // and will do a tighter radius than the outside wheel.
                                if (vehicleSide == math.sign(wheelOnVehicle.SteerAngle))
                                {
                                    float steerAnglePercent =
                                        math.abs(wheelOnVehicle.SteerAngle) / wheelControl.MaxSteerAngle;
                                    wheelOnVehicle.SteerAngle *= math.lerp(1f, 1f + wheelControl.AckermanCoefficient,
                                        steerAnglePercent);
                                }
                            }
                        }

                        // Disable static friction on all wheels if there is throttle applied
                        if (vehicleControlData.ThrottlePosition > 0f)
                        {
                            wheelOnVehicle.DisableStaticFrictionSingleFrame = true;
                        }

                        // Assign motor torque
                        if (isEngineConnectedToWheels && wheelControl.MotorTorqueCoefficient != 0 &&
                            drivenWheelMotorTorqueCoefficientSum > 0)
                        {
                            // Split the transmission output torque based on the motor torque coefficients on the wheels.
                            // This allows for motor torque bias between axles without actual differentials.
                            wheelOnVehicle.MotorTorque =
                                (wheelControl.MotorTorqueCoefficient / drivenWheelMotorTorqueCoefficientSum) *
                                transmissionTorque;
                        }
                        else
                        {
                            wheelOnVehicle.MotorTorque = 0f;
                        }

                        // Assign brake and hanbrake torque.
                        float brakeTorque = vehicleControlData.BrakePosition * vehicleControlData.MaxBrakeTorque *
                                            wheelControl.BrakeTorqueCoefficient;
                        float handbrakeTorque = vehicleControl.HandbrakeInput * vehicleControlData.MaxBrakeTorque *
                                                wheelControl.HandbrakeTorqueCoefficient;
                        wheelOnVehicle.BrakeTorque = math.max(brakeTorque, handbrakeTorque);
                        
                        // Spin resistance
                        if(math.abs(wheelOnVehicle.MotorTorque) <= math.EPSILON)
                        {
                            wheelOnVehicle.BrakeTorque = math.max(wheelOnVehicle.BrakeTorque, wheelControl.SpinResistanceTorque);
                        }

                        // Speed limiting based on vehicle speed limit and engine rev limiter
                        float wheelAngularVelocityLimit = vehicleControlData.WheelAngularVelocityLimit <= 0f
                            ? math.INFINITY
                            : vehicleControlData.WheelAngularVelocityLimit / wheel.Radius;
                        float powertrainAngularVelocityLimit =
                            isEngineConnectedToWheels && wheelControl.MotorTorqueCoefficient != 0
                                ? vehicleControlData.EngineMaxAngularVelocity /
                                  math.abs(vehicleControlData.TransmissionCurrentGearRatio)
                                : math.INFINITY;
                        wheelOnVehicle.AngularVelocityLimit =
                            math.min(wheelAngularVelocityLimit, powertrainAngularVelocityLimit);

                        vehicleWheels[wheelIndex] = wheelOnVehicle;
                    }
                }

                // Update aerodynamics
                if (vehicleControlData.AeroDragCoefficient > 0 || vehicleControlData.AeroDownforceCoefficient > 0)
                {
                    // Drag
                    float3 area;

                    // Calculate frontal area assuming a rectangular frontal profile
                    area.z = vehicle.Dimensions.x * vehicle.Dimensions.y;

                    // Calculate side area, only relevant when the vehicle is going sideways
                    area.x = vehicle.Dimensions.y * vehicle.Dimensions.z;

                    // Calculate top-down area, could be relevant when falling down
                    area.y = vehicle.Dimensions.x * vehicle.Dimensions.z;

                    // Calculate aerodynamic drag
                    if (vehicleControlData.AeroDragCoefficient > 0)
                    {
                        // 0.5 * RHO = 0.6125
                        float c = -0.6125f * vehicleControlData.AeroDragCoefficient;

                        vehicleControlData.AeroDragImpulse = new float3(
                            c * area.x * (localVelocity.x * localVelocity.x) * math.sign(localVelocity.x),
                            c * area.y * (localVelocity.y * localVelocity.y) * math.sign(localVelocity.y),
                            c * area.z * (localVelocity.z * localVelocity.z) * math.sign(localVelocity.z)
                            ) * DeltaTime;
                        vehicleControlData.AeroDragPosition = transform.TransformPoint(physicsMass.CenterOfMass + vehicleControlData.AeroDragOffset);
                        physicsVelocity.ApplyImpulse(in physicsMass, in transform.Position, in transform.Rotation,
                            transform.Scale, vehicleControlData.AeroDragImpulse, vehicleControlData.AeroDragPosition);
                    }

                    // Calculate downforce
                    if (vehicleControlData.AeroDownforceCoefficient > 0)
                    {
                        // Assumes that downforce is the same in both directions, and that there is no negative downforce (lift).
                        // We do not have any wing info so use the vehicle top-down projection surface area to scale the value with
                        // vehicle size.
                        vehicleControlData.AeroDownforceImpulse = -0.6125f * vehicleControlData.AeroDownforceCoefficient * area.y * (localVelocity.z
                            * localVelocity.z) * transform.Up() * DeltaTime;
                        vehicleControlData.AeroDownforcePosition = transform.TransformPoint(physicsMass.CenterOfMass + vehicleControlData.AeroDownforceOffset);
                        physicsVelocity.ApplyImpulse(in physicsMass, in transform.Position, in transform.Rotation,
                            transform.Scale, vehicleControlData.AeroDownforceImpulse, vehicleControlData.AeroDownforcePosition);
                    }
                }
            }
           
            /// <summary>
            /// Starts the engine.
            /// </summary>
            /// <param name="entity">Target <see cref="VehicleControl"/></param>
            public void StartEngine(Entity entity)
            {
                if (EngineControlLookup.HasComponent(entity))
                {
                    RefRW<EngineStartStop> engineControlRW = EngineControlLookup.GetRefRW(entity);
                    engineControlRW.ValueRW.Start = true;
                    EngineControlLookup.SetComponentEnabled(entity, true);
                }
            }

            /// <summary>
            /// Stops the engine.
            /// </summary>
            /// <param name="entity">Target <see cref="VehicleControl"/>.</param>
            public void StopEngine(Entity entity)
            {
                if (EngineControlLookup.HasComponent(entity))
                {
                    RefRW<EngineStartStop> engineControlRW = EngineControlLookup.GetRefRW(entity);
                    engineControlRW.ValueRW.Start = false;
                    EngineControlLookup.SetComponentEnabled(entity, true);
                }
            }

            /// <summary>
            /// Attempts to shift the transmission up a gear. 
            /// </summary>
            /// <param name="vehicleControlData">Target <see cref="VehicleControlData"/>.</param>
            /// <param name="controlEvents">Vehicle control inputs.</param>
            /// <returns>True on successful shift.</returns>
            public static bool ShiftUp(ref VehicleControlData vehicleControlData, ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                return ShiftInto(vehicleControlData.TransmissionCurrentGear + 1, ref vehicleControlData, ref controlEvents);
            }

            /// <summary>
            /// Attempts to shift the transmission down a gear. 
            /// </summary>
            /// <param name="vehicleControlData">Target <see cref="VehicleControlData"/>.</param>
            /// <param name="controlEvents">Vehicle control inputs.</param>
            /// <returns>True on successful shift.</returns>
            public static bool ShiftDown(ref VehicleControlData vehicleControlData, ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                return ShiftInto(vehicleControlData.TransmissionCurrentGear - 1, ref vehicleControlData, ref controlEvents);
            }

            /// <summary>
            /// Attempts to shift the transmission into the target gear.
            /// </summary>
            /// <param name="targetGear">Target fear to shift into.</param>
            /// <param name="vehicleControlData">Target <see cref="VehicleControlData"/>.</param>
            /// <param name="controlEvents">Vehicle control inputs.</param>
            /// <returns>True on successful shift.</returns>
            public static bool ShiftInto(int targetGear, ref VehicleControlData vehicleControlData, ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                int currentGear = vehicleControlData.TransmissionCurrentGear;
                if (targetGear == currentGear)
                {
                    return false;
                }

                // Shift into N
                if (targetGear == 0)
                {
                    vehicleControlData.TransmissionCurrentGearRatio = 0;
                }
                // Shift into D
                else if (targetGear > 0)
                {
                    // Index of D1 gear will be 0 in the gear ratios list
                    int gearListIndex = targetGear - 1;
                    int forwardGearCount = vehicleControlData.TransmissionForwardGearRatios.Length;
                    if (gearListIndex >= forwardGearCount)
                    {
                        return false;
                    }
                    vehicleControlData.TransmissionCurrentGearRatio = vehicleControlData.TransmissionForwardGearRatios[gearListIndex]
                                                                      * vehicleControlData.TransmissionFinalGearRatio;
                }
                // Shift into R 
                else
                {
                    // Index of R1 gear will be 0 in the gear ratios list
                    int gearListIndex = -(targetGear + 1);
                    int reverseGearCount = vehicleControlData.TransmissionReverseGearRatios.Length;
                    if (gearListIndex >= reverseGearCount)
                    {
                        return false;
                    }
                    vehicleControlData.TransmissionCurrentGearRatio = vehicleControlData.TransmissionReverseGearRatios[gearListIndex]
                                                                      * vehicleControlData.TransmissionFinalGearRatio;
                }

                // Add events for effects, etc.
                if (currentGear > targetGear)
                {
                    controlEvents.Add(new VehicleControlEvent(VehicleControlEvent.Type.ShiftUp));
                }
                else
                {
                    controlEvents.Add(new VehicleControlEvent(VehicleControlEvent.Type.ShiftDown));
                }

                vehicleControlData.TransmissionCurrentGear = targetGear;
                return true;
            }

            /// <summary>
            /// Handles gear shifts automatically.
            /// </summary>
            /// <param name="speed">Vehicle speed.</param>
            /// <param name="wheelRadius">Reference wheel radius.</param>
            /// &lt;param name="vehicleControl"&gt;Target &lt;see cref="VehicleControl"/&gt;.&lt;/param&gt;
            /// <param name="vehicleControlData">Target <see cref="VehicleControlData"/>.</param>
            /// <param name="vehicle">Target <see cref="Vehicle"/></param>
            /// <param name="controlEvents"><see cref="VehicleControlEvent"/></param>
            private void AutoShift(in float speed, in float wheelRadius, in VehicleControl vehicleControl, ref VehicleControlData vehicleControlData, ref Vehicle vehicle, ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                // Using raw input here since we care about the user intent to go forward or reverse
                // instead of the actual throttle and brake values that are applied to the powertrain.

                // Calculate the speed at which the transmission will shift. This is intentionally not based on the angular velocity as 
                // we could have wheel spin which would make the transmission shift sooner than it should.
                float invGearRatio = 1f / vehicleControlData.TransmissionCurrentGearRatio;

                float upshiftSpeed = vehicleControlData.TransmissionShiftUpAngularVelocity * invGearRatio * wheelRadius;
                float downshiftSpeed = vehicleControlData.TransmissionShiftDownAngularVelocity * invGearRatio * wheelRadius;

                // Currently in neutral
                // Shift to D/R if there is intent or if the vehicle is moving fast enough that it should not be in neutral.
                if (vehicleControlData.TransmissionCurrentGear == 0)
                {
                    float directionChangeSpeedThreshold = 0.4f * vehicle.VirtualScale;
                    if (speed > -directionChangeSpeedThreshold && vehicleControl.RawThrottleInput > 0f)
                    {
                        ShiftUp(ref vehicleControlData, ref controlEvents);
                    }
                    else if (speed < directionChangeSpeedThreshold && vehicleControl.RawBrakeInput > 0f)
                    {
                        ShiftDown(ref vehicleControlData, ref controlEvents);
                    }
                }
                else
                {
                    // Invalid state, non neutral gears should not have 0 gear ratio.
                    if (vehicleControlData.TransmissionCurrentGearRatio == 0f)
                    {
                        return;
                    }

                    // Currently in forward gears
                    if (vehicleControlData.TransmissionCurrentGear > 0)
                    {
                        // Try to upshift
                        if (speed >= upshiftSpeed)
                        {
                            // Normal upshift
                            ShiftUp(ref vehicleControlData, ref controlEvents);
                        }
                        // Try to downshift
                        else if (speed < downshiftSpeed)
                        {
                            if (vehicleControlData.TransmissionCurrentGear == 1
                                && vehicleControl.RawThrottleInput == 0f
                                && vehicleControlData.RelativeForwardSpeed < 1f * vehicle.VirtualScale
                                && vehicleControlData.EngineAngularVelocity <= vehicleControlData.EngineIdleAngularVelocity)
                            {
                                // Not trying to accelerate, go into N.
                                ShiftDown(ref vehicleControlData, ref controlEvents);
                            }
                            else if (vehicleControlData.TransmissionCurrentGear > 1)
                            {
                                // Normal downshift
                                ShiftDown(ref vehicleControlData, ref controlEvents);
                            }
                        }
                    }
                    // Currently in reverse gears.
                    // Downshift and upshift switch roles since upshifting in reverse is going lower into negative gears,
                    // e.g. from -1 to -2 (R1 to R2) where R2 has lower gear ratio.
                    else if (vehicleControlData.TransmissionCurrentGear < 0)
                    {
                        // Try to reverse downshift (e.g. R1 to R2)
                        if (speed <= upshiftSpeed)
                        {
                            ShiftDown(ref vehicleControlData, ref controlEvents);
                        }
                        // Try to reverse upshift (e.g. R2 to R1, or R1 to N)
                        else if (speed > downshiftSpeed)
                        {
                            if (vehicleControlData.TransmissionCurrentGear == -1
                                && vehicleControl.RawBrakeInput == 0f
                                && vehicleControlData.RelativeForwardSpeed > -1f * vehicle.VirtualScale)
                            {
                                // Not trying to accelerate in reverse, go into N.
                                ShiftUp(ref vehicleControlData, ref controlEvents);
                            }
                            else if (vehicleControlData.TransmissionCurrentGear < -1)
                            {
                                ShiftUp(ref vehicleControlData, ref controlEvents);
                            }
                        }
                    }
                }
            }

            /// <summary>
            /// Manual transmission shift handling.
            /// </summary>
            /// <param name="vehicleControl">Target <see cref="VehicleControl"/>.</param>
            /// <param name="vehicleControlData">Target <see cref="VehicleControlData"/>.<param>
            /// <param name="controlEvents"><see cref="VehicleControlEvent"/>.</param>
            private void ManualShift(in VehicleControl vehicleControl, ref VehicleControlData vehicleControlData, ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                if (vehicleControl.ShiftUpInput)
                {
                    ShiftUp(ref vehicleControlData, ref controlEvents);
                }
                else if (vehicleControl.ShiftDownInput)
                {
                    ShiftDown(ref vehicleControlData, ref controlEvents);
                }
            }
        }

        /// <summary>
        /// Controls engine starting and stopping, based on the state of <see cref="EngineStartStop"/>.
        /// To start or stop the engine, enable the <see cref="EngineStartStop"/> component and set the 
        /// value of the Start field to the desired engine running state.
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        [WithAll(typeof(EngineStartStop))]
        public partial struct VehicleEngineControlJob : IJobEntity
        {
            /// <summary>
            /// Lookup for the <see cref="EngineStartStop"/> Entity.
            /// </summary>
            public ComponentLookup<EngineStartStop> EngineControlLookup;

            void Execute(
                Entity entity,
                ref VehicleControlData vehicleControlData,
                ref DynamicBuffer<VehicleControlEvent> controlEvents)
            {
                EngineStartStop engineStartStop = EngineControlLookup[entity];
                if (engineStartStop.Start)
                {
                    if (!vehicleControlData.EngineIsRunning)
                    {
                        vehicleControlData.EngineIsRunning = true;
                        controlEvents.Add(new VehicleControlEvent(VehicleControlEvent.Type.StartEngine));
                    }
                }
                else
                {
                    if (vehicleControlData.EngineIsRunning)
                    {
                        vehicleControlData.EngineIsRunning = false;
                        vehicleControlData.EngineAngularVelocity = 0f;
                        controlEvents.Add(new VehicleControlEvent(VehicleControlEvent.Type.StopEngine));
                    }
                }

                EngineControlLookup.SetComponentEnabled(entity, false);
            }
        }
    }
}