using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Transforms;
using UnityEngine;

namespace Unity.Vehicles
{
    /// <summary>
    /// Various vehicle utilities
    /// </summary>
    public static class VehicleUtilities
    {
        /// <summary>
        /// Draws a wheel gizmo indicating the possible suspension travel and the
        /// current wheel position.
        /// </summary>
        /// <param name="suspensionTransform">World <see cref="RigidTransform"/> representing the suspension origin.</param>
        /// <param name="wheelTransform">World <see cref="RigidTransform"/> representing the wheel.</param>
        /// <param name="radius">Radius of the wheel.</param>
        /// <param name="width">Width of the wheel.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void DrawWheelGizmo(in RigidTransform suspensionTransform, in RigidTransform wheelTransform,
            float radius, float width, float suspensionMaxLength)
        {
            Vector3 right = math.mul(suspensionTransform.rot, math.right());
            Vector3 forward = math.mul(suspensionTransform.rot, math.forward());
            Vector3 up = math.mul(suspensionTransform.rot, math.up());

            Vector3 center = wheelTransform.pos;
            Vector3 centerLeft = center - right * width * 0.5f;
            Vector3 centerRight = center + right * width * 0.5f;

            Vector3 centerCompressed = suspensionTransform.pos;
            Vector3 centerLeftCompressed = centerCompressed - right * width * 0.5f;
            Vector3 centerRightCompressed = centerCompressed + right * width * 0.5f;

            Vector3 centerExtended = (Vector3)(suspensionTransform.pos) - up * suspensionMaxLength;
            Vector3 centerLeftExtended = centerExtended - right * width * 0.5f;
            Vector3 centerRightExtended = centerExtended + right * width * 0.5f;

            // Draw the wheel travel
            Gizmos.color = Color.grey;
            Vector3 forwardOffset = forward * radius;
            Vector3 upOffset = up * radius;
            Gizmos.DrawLine(centerLeftCompressed - forwardOffset, centerLeftExtended - forwardOffset);
            Gizmos.DrawLine(centerLeftCompressed + forwardOffset, centerLeftExtended + forwardOffset);
            Gizmos.DrawLine(centerRightCompressed - forwardOffset, centerRightExtended - forwardOffset);
            Gizmos.DrawLine(centerRightCompressed + forwardOffset, centerRightExtended + forwardOffset);
            Gizmos.DrawLine(centerLeftCompressed - forwardOffset, centerRightCompressed - forwardOffset);
            Gizmos.DrawLine(centerLeftCompressed + forwardOffset, centerRightCompressed + forwardOffset);
            Gizmos.DrawLine(centerLeftExtended - forwardOffset, centerRightExtended - forwardOffset);
            Gizmos.DrawLine(centerLeftExtended + forwardOffset, centerRightExtended + forwardOffset);
            Gizmos.DrawLine(centerLeftCompressed + upOffset, centerRightCompressed + upOffset);
            Gizmos.DrawLine(centerLeftExtended - upOffset, centerRightExtended - upOffset);
            DrawWireArcGizmo(centerLeftCompressed, right, forward, -180, radius);
            DrawWireArcGizmo(centerRightCompressed, right, forward, -180, radius);
            DrawWireArcGizmo(centerLeftExtended, right, forward, 180, radius);
            DrawWireArcGizmo(centerRightExtended, right, forward, 180, radius);

            // Draw the current wheel position
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(center, radius * 0.08f);
            DrawWireDiscGizmo(centerRight, right, forward, radius);
            DrawWireDiscGizmo(centerLeft, right, forward, radius);

            // Draw the spring travel
            Gizmos.color = Color.yellow;
            Vector3 horizontalOffset = forward * 0.2f * radius;
            Gizmos.DrawLine(centerCompressed + horizontalOffset, centerCompressed - horizontalOffset);
            Gizmos.DrawLine(centerExtended + horizontalOffset, centerExtended - horizontalOffset);
            Gizmos.DrawLine(centerCompressed, centerExtended);
        }

        private static void DrawWireDiscGizmo(Vector3 center, Vector3 normal, Vector3 forward, float radius, int segments = 24)
        {
            float step = 360.0f / segments;
            Vector3 prevPoint = center + Quaternion.AngleAxis(0, normal) * (forward * radius);

            for (int i = 0; i <= segments; i++)
            {
                float angle = i * step;
                Vector3 nextPoint = center + Quaternion.AngleAxis(angle, normal) * (forward * radius);
                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }
        }

        private static void DrawWireArcGizmo(Vector3 center, Vector3 normal, Vector3 forward, float angleRange, float radius, int segments = 12)
        {
            float step = angleRange / segments;
            Vector3 prevPoint = center + Quaternion.AngleAxis(0, normal) * (forward * radius);

            for (int i = 0; i <= segments; i++)
            {
                float angle = i * step;
                Vector3 nextPoint = center + Quaternion.AngleAxis(angle, normal) * (forward * radius);
                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }
        }

        /// <summary>
        /// Returns the velocity of a hit rigidbody at the hit point
        /// </summary>
        /// <param name="physicsWorld"> The PhysicsWorld</param>
        /// <param name="hit"> The hit </param>
        /// <param name="hitBodyVelocity"> The calculated velocity of the hit body </param>
        public static void GetHitBodyVelocity(
            in PhysicsWorld physicsWorld,
            in ColliderCastHit hit,
            out float3 hitBodyVelocity)
        {
            hitBodyVelocity = float3.zero;

            if (hit.Entity != Entity.Null &&
                IsBodyNonStatic(physicsWorld, hit.RigidBodyIndex))
            {
                hitBodyVelocity =
                    physicsWorld.GetLinearVelocity(hit.RigidBodyIndex, hit.Position);
            }
        }

        /// <summary>
        /// Returns true if the Rigidbody is dynamic.
        /// </summary>
        /// <param name="physicsWorld">Physics world.</param>
        /// <param name="rigidbodyIndex">Index of the body.</param>
        /// <returns>True if dynamic (non-kinematic) Rigidbody.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsBodyDynamic(in PhysicsWorld physicsWorld, int rigidbodyIndex)
        {
            if (rigidbodyIndex < physicsWorld.NumDynamicBodies)
            {
                if (physicsWorld.MotionVelocities[rigidbodyIndex].InverseMass > 0f)
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Returns true if the body is a Rigidbody, dynamic or kinematic.
        /// </summary>
        /// <param name="physicsWorld">Physics world.</param>
        /// <param name="rigidbodyIndex">Index of the body.</param>
        /// <returns>True if Rigidbody.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsBodyNonStatic(in PhysicsWorld physicsWorld, int rigidbodyIndex)
        {
            if (rigidbodyIndex < physicsWorld.NumDynamicBodies)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Tries to find a vehicle wheel on the vehicle, based on a wheel entity.
        /// </summary>
        /// <param name="wheelEntity">The wheel entity to find</param>
        /// <param name="wheelsOnVehicleBuffer">The vehicle wheels buffer on the target vehicle</param>
        /// <param name="wheelOnVehicle">The found vehicle wheel</param>
        /// <param name="vehicleWheelIndex">The found vehicle wheel index</param>
        /// <returns>True if wheel was found</returns>
        public static bool TryGetWheelOnVehicle(Entity wheelEntity,
            in DynamicBuffer<WheelOnVehicle> wheelsOnVehicleBuffer,
            out WheelOnVehicle wheelOnVehicle,
            out int vehicleWheelIndex)
        {
            for (int i = 0; i < wheelsOnVehicleBuffer.Length; i++)
            {
                WheelOnVehicle tmpWheelOnVehicle = wheelsOnVehicleBuffer[i];
                if (tmpWheelOnVehicle.Entity == wheelEntity)
                {
                    wheelOnVehicle = tmpWheelOnVehicle;
                    vehicleWheelIndex = i;
                    return true;
                }
            }

            wheelOnVehicle = default;
            vehicleWheelIndex = -1;
            return false;
        }

        /// <summary>
        /// Tries to add a wheel to the vehicle.
        /// </summary>
        /// <param name="vehicleEntity">Vehicle entity to which to add te wheel to.</param>
        /// <param name="wheelEntity">Wheel which is to be added.</param>
        /// <param name="pairedWheelEntity">Wheel entity that is sharing the same axle with the wheel to be added (if any).</param>
        /// <param name="wheelLocalTransformOnVehicle">Local <see cref="RigidTransform"/> representing the wheel position.</param>
        /// <param name="wheelLookup"><see cref="Wheel"/> lookup.</param>
        /// <param name="wheelOnVehicleBufferLookup"><see cref="WheelOnVehicle"/> lookup.</param>
        /// <param name="wheelOnVehicleReferenceLookup"><see cref="WheelOnVehicleReference"/> lookup.</param>
        /// <param name="vehicleUpdateCollidersLookup"><see cref="VehicleUpdateColliders"/> lookup.</param>
        /// <param name="vehicleUpdateWheelDataLookup"><see cref="VehicleUpdateWheelData"/> lookup.</param>
        /// <param name="vehicleWheelEventsLookup"><see cref="VehicleWheelEvent"/> lookup.</param>
        /// <returns>True if successful.</returns>
        public static bool TryAddWheel(
            Entity vehicleEntity,
            Entity wheelEntity,
            Entity pairedWheelEntity,
            in RigidTransform wheelLocalTransformOnVehicle,
            in ComponentLookup<Wheel> wheelLookup,
            ref BufferLookup<WheelOnVehicle> wheelOnVehicleBufferLookup,
            ref ComponentLookup<WheelOnVehicleReference> wheelOnVehicleReferenceLookup,
            ref ComponentLookup<VehicleUpdateColliders> vehicleUpdateCollidersLookup,
            ref ComponentLookup<VehicleUpdateWheelData> vehicleUpdateWheelDataLookup,
            ref BufferLookup<VehicleWheelEvent> vehicleWheelEventsLookup)
        {
            if (wheelOnVehicleBufferLookup.TryGetBuffer(vehicleEntity, out DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer))
            {
                if (wheelLookup.TryGetComponent(wheelEntity, out Wheel wheel))
                {
                    WheelOnVehicle wheelOnVehicle = WheelOnVehicle.GetDefault();
                    wheelOnVehicle.Entity = wheelEntity;
                    wheelOnVehicle.SuspensionLocalTransform = wheelLocalTransformOnVehicle;
                    wheelOnVehicle.Wheel = wheel;
                    wheelOnVehicle.AxlePairedWheelEntity = pairedWheelEntity;
                    vehicleWheelsBuffer.Add(wheelOnVehicle);

                    if (vehicleUpdateCollidersLookup.HasComponent(vehicleEntity))
                    {
                        EnabledRefRW<VehicleUpdateColliders> vehicleUpdateCollidersEnabled =
                            vehicleUpdateCollidersLookup.GetEnabledRefRW<VehicleUpdateColliders>(vehicleEntity);
                        vehicleUpdateCollidersEnabled.ValueRW = true;
                    }

                    if (vehicleUpdateWheelDataLookup.HasComponent(vehicleEntity))
                    {
                        EnabledRefRW<VehicleUpdateWheelData> vehicleUpdateWheelDataEnabled =
                            vehicleUpdateWheelDataLookup.GetEnabledRefRW<VehicleUpdateWheelData>(vehicleEntity);
                        vehicleUpdateWheelDataEnabled.ValueRW = true;
                    }

                    if (vehicleWheelEventsLookup.TryGetBuffer(vehicleEntity, out DynamicBuffer<VehicleWheelEvent> vehicleEventsBuffer))
                    {
                        vehicleEventsBuffer.Add(new VehicleWheelEvent
                        {
                            wheelEventType = VehicleWheelEvent.VehicleWheelEventType.WheelAdded,
                            WheelEntity = wheelEntity,
                            WheelSuspensionLocalTransform = wheelLocalTransformOnVehicle,
                            WheelAngularVelocity = 0f,
                        });
                    }

                    // Following the addition of the wheel, update indices
                    UpdateWheelIndices(vehicleEntity, ref wheelOnVehicleBufferLookup, ref wheelOnVehicleReferenceLookup);

                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Removes a wheel from a vehicle
        /// </summary>
        /// <param name="vehicleEntity"> The vehicle entity</param>
        /// <param name="wheelEntity"> The wheel entity to remove</param>
        /// <param name="wheelOnVehicleBufferLookup"> The <see cref="WheelOnVehicle"/> buffer lookup </param>
        /// <param name="wheelOnVehicleReferenceLookup"> The <see cref="WheelOnVehicleReference"/> component lookup </param>
        /// <param name="vehicleUpdateCollidersLookup">The <see cref="VehicleUpdateColliders"/> component lookup </param>
        /// <param name="vehicleUpdateWheelDataLookup">The <see cref="VehicleUpdateWheelData"/> component lookup </param>
        /// <param name="vehicleWheelEventsLookup">The <see cref="VehicleWheelEvent"/> component lookup </param>
        /// <param name="removedWheel"> The removed wheel data</param>
        /// <returns>True if successful.</returns>
        public static bool TryRemoveWheel(
            Entity vehicleEntity,
            Entity wheelEntity,
            ref BufferLookup<WheelOnVehicle> wheelOnVehicleBufferLookup,
            ref ComponentLookup<WheelOnVehicleReference> wheelOnVehicleReferenceLookup,
            ref ComponentLookup<VehicleUpdateColliders> vehicleUpdateCollidersLookup,
            ref ComponentLookup<VehicleUpdateWheelData> vehicleUpdateWheelDataLookup,
            ref BufferLookup<VehicleWheelEvent> vehicleWheelEventsLookup,
            out WheelOnVehicle removedWheel)
        {
            removedWheel = default;

            if (wheelOnVehicleBufferLookup.TryGetBuffer(vehicleEntity,
                    out DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer))
            {
                for (int wheelIndex = 0; wheelIndex < vehicleWheelsBuffer.Length; wheelIndex++)
                {
                    var tmpWheel = vehicleWheelsBuffer[wheelIndex];
                    if (tmpWheel.Entity == wheelEntity)
                    {
                        return TryRemoveWheel(
                            vehicleEntity,
                            wheelIndex,
                            ref wheelOnVehicleBufferLookup,
                            ref wheelOnVehicleReferenceLookup,
                            ref vehicleUpdateCollidersLookup,
                            ref vehicleUpdateWheelDataLookup,
                            ref vehicleWheelEventsLookup,
                            out removedWheel);
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Removes a wheel from a vehicle
        /// </summary>
        /// <param name="vehicleEntity"> The vehicle entity</param>
        /// <param name="wheelIndex"> The index of the wheel to remove on the vehicle</param>
        /// <param name="wheelOnVehicleBufferLookup"> The <see cref="WheelOnVehicle"/> buffer lookup </param>
        /// <param name="wheelOnVehicleReferenceLookup"> The <see cref="WheelOnVehicleReference"/> component lookup </param>
        /// <param name="vehicleUpdateCollidersLookup">The <see cref="VehicleUpdateColliders"/> component lookup </param>
        /// <param name="vehicleUpdateWheelDataLookup">The <see cref="VehicleUpdateWheelData"/> component lookup </param>
        /// <param name="vehicleWheelEventsLookup">The <see cref="VehicleWheelEvent"/> component lookup </param>
        /// <param name="removedWheel"> The removed wheel data</param>
        /// <returns>True if successful.</returns>
        public static bool TryRemoveWheel(
            Entity vehicleEntity,
            int wheelIndex,
            ref BufferLookup<WheelOnVehicle> wheelOnVehicleBufferLookup,
            ref ComponentLookup<WheelOnVehicleReference> wheelOnVehicleReferenceLookup,
            ref ComponentLookup<VehicleUpdateColliders> vehicleUpdateCollidersLookup,
            ref ComponentLookup<VehicleUpdateWheelData> vehicleUpdateWheelDataLookup,
            ref BufferLookup<VehicleWheelEvent> vehicleWheelEventsLookup,
            out WheelOnVehicle removedWheel)
        {
            removedWheel = default;

            if (wheelOnVehicleBufferLookup.TryGetBuffer(vehicleEntity,
                    out DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer))
            {
                if (wheelIndex >= 0 && wheelIndex < vehicleWheelsBuffer.Length)
                {
                    removedWheel = vehicleWheelsBuffer[wheelIndex];

                    // Clear wheel reference
                    if (wheelOnVehicleReferenceLookup.HasComponent(removedWheel.Entity))
                    {
                        wheelOnVehicleReferenceLookup[removedWheel.Entity] = new WheelOnVehicleReference
                        {
                            VehicleEntity = Entity.Null,
                            Index = -1,
                        };
                    }

                    if (vehicleUpdateCollidersLookup.HasComponent(vehicleEntity))
                    {
                        EnabledRefRW<VehicleUpdateColliders> vehicleUpdateCollidersEnabled =
                            vehicleUpdateCollidersLookup.GetEnabledRefRW<VehicleUpdateColliders>(vehicleEntity);
                        vehicleUpdateCollidersEnabled.ValueRW = true;
                    }

                    if (vehicleUpdateWheelDataLookup.HasComponent(vehicleEntity))
                    {
                        EnabledRefRW<VehicleUpdateWheelData> vehicleUpdateWheelDataEnabled =
                            vehicleUpdateWheelDataLookup.GetEnabledRefRW<VehicleUpdateWheelData>(vehicleEntity);
                        vehicleUpdateWheelDataEnabled.ValueRW = true;
                    }

                    if (vehicleWheelEventsLookup.TryGetBuffer(vehicleEntity, out DynamicBuffer<VehicleWheelEvent> vehicleEventsBuffer))
                    {
                        vehicleEventsBuffer.Add(new VehicleWheelEvent
                        {
                            wheelEventType = VehicleWheelEvent.VehicleWheelEventType.WheelRemoved,
                            WheelEntity = removedWheel.Entity,
                            WheelSuspensionLocalTransform = removedWheel.SuspensionLocalTransform,
                            WheelAngularVelocity = removedWheel.AngularVelocity,
                        });
                    }

                    vehicleWheelsBuffer.RemoveAt(wheelIndex);

                    // Following the removal of the wheel, update indices
                    UpdateWheelIndices(vehicleEntity, ref wheelOnVehicleBufferLookup, ref wheelOnVehicleReferenceLookup);

                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// For each wheel, update the vehicle entity and wheel index on the wheel entity.
        /// And update the Axle paired wheels index in the vehicle wheel buffer.
        /// </summary>
        private static void UpdateWheelIndices(
            Entity vehicleEntity,
            ref BufferLookup<WheelOnVehicle> vehicleWheelBufferLookup,
            ref ComponentLookup<WheelOnVehicleReference> vehicleWheelReferenceLookup)
        {
            if (vehicleWheelBufferLookup.TryGetBuffer(vehicleEntity,
                    out DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer))
            {
                for (int i = 0; i < vehicleWheelsBuffer.Length; i++)
                {
                    WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[i];

                    // Update cached index on wheel entity
                    if (vehicleWheelReferenceLookup.TryGetComponent(wheelOnVehicle.Entity, out WheelOnVehicleReference vehicleWheelReference))
                    {
                        vehicleWheelReference.VehicleEntity = vehicleEntity;
                        vehicleWheelReference.Index = i;
                        vehicleWheelReferenceLookup[wheelOnVehicle.Entity] = vehicleWheelReference;
                    }

                    // Search for paired wheel index
                    wheelOnVehicle.AxlePairedWheelIndex = -1;
                    for (int j = 0; j < vehicleWheelsBuffer.Length; j++)
                    {
                        if (j != i)
                        {
                            WheelOnVehicle otherWheelOnVehicle = vehicleWheelsBuffer[j];
                            if (otherWheelOnVehicle.Entity == wheelOnVehicle.AxlePairedWheelEntity)
                            {
                                wheelOnVehicle.AxlePairedWheelIndex = j;
                                break;
                            }
                        }
                    }

                    vehicleWheelsBuffer[i] = wheelOnVehicle;
                }
            }
        }


        /// <summary>
        /// Calculates the local transform of the wheel's suspension with steering taken into account.
        /// </summary>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <returns>World transform of the wheel's suspension.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform GetSteeredSuspensionLocalTransform(in WheelOnVehicle wheelOnVehicle)
        {
            RigidTransform wheelSteeringLocalRotationTransform = new RigidTransform(quaternion.RotateY(wheelOnVehicle.SteerAngle), float3.zero);
            return math.mul(wheelOnVehicle.SuspensionLocalTransform, wheelSteeringLocalRotationTransform);
        }

        /// <summary>
        /// Calculates the world transform of the wheel's suspension.
        /// </summary>
        /// <param name="vehicleTransform">The transform of the vehicle the suspension belongs to.</param>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <returns>World transform of the wheel's suspension.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform GetSteeredSuspensionWorldTransform(in RigidTransform vehicleTransform, in WheelOnVehicle wheelOnVehicle)
        {
            return math.mul(vehicleTransform, GetSteeredSuspensionLocalTransform(in wheelOnVehicle));
        }

        /// <summary>
        /// Calculates the local transform of the wheel.
        /// </summary>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <returns>Local transform of the wheel.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform GetWheelLocalTransform(in WheelOnVehicle wheelOnVehicle)
        {
            RigidTransform wheelLocalTransform = GetSteeredSuspensionLocalTransform(in wheelOnVehicle);
            wheelLocalTransform.pos += GetSuspensionLocalDirection(in wheelOnVehicle) * wheelOnVehicle.SuspensionLength;
            return wheelLocalTransform;
        }
        /// <summary>
        /// Calculates the world transform of the wheel.
        /// </summary>
        /// <param name="vehicleTransform">The transform of the vehicle the suspension belongs to.</param>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <returns>World transform of the wheel. Note that this transform does not include any spin around the axle.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform GetWheelWorldTransform(in RigidTransform vehicleTransform, in WheelOnVehicle wheelOnVehicle)
        {
            return math.mul(vehicleTransform, GetWheelLocalTransform(in wheelOnVehicle));
        }

        /// <summary>
        /// Calculates the world transform of the wheel.
        /// </summary>
        /// <param name="steeredSuspensionWorldTransform"> The steered suspension world transform </param>
        /// <param name="suspensionLength"> The current length of the suspension </param>
        /// <returns>The world transform of the wheel </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform GetWheelWorldTransform(in RigidTransform steeredSuspensionWorldTransform, float suspensionLength)
        {
            RigidTransform wheelWorldTransform = steeredSuspensionWorldTransform;
            wheelWorldTransform.pos +=
                GetSuspensionWorldDirection(in steeredSuspensionWorldTransform) * suspensionLength;
            return wheelWorldTransform;
        }

        /// <summary>
        /// Gets the local direction of the suspension
        /// </summary>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <returns>The local direction of the suspension</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetSuspensionLocalDirection(in WheelOnVehicle wheelOnVehicle)
        {
            return math.mul(wheelOnVehicle.SuspensionLocalTransform.rot, math.down());
        }

        /// <summary>
        /// Gets the world direction of the suspension
        /// </summary>
        /// <param name="vehicleTransform">The transform of the vehicle the suspension belongs to.</param>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <returns>The world direction of the suspension</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetSuspensionWorldDirection(in RigidTransform vehicleTransform, in WheelOnVehicle wheelOnVehicle)
        {
            return math.mul(vehicleTransform.rot, GetSuspensionLocalDirection(in wheelOnVehicle));
        }

        /// <summary>
        /// Gets the world direction of the suspension
        /// </summary>
        /// <param name="steeredSuspensionWorldTransform"> The steered suspension world transform </param>
        /// <returns> The world direction of the suspension </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetSuspensionWorldDirection(in RigidTransform steeredSuspensionWorldTransform)
        {
            return math.mul(steeredSuspensionWorldTransform.rot, math.down());
        }

        /// <summary>
        /// Gets the wheel right direction
        /// </summary>
        /// <param name="steeredSuspensionTransform"> The steered suspension transform </param>
        /// <returns>The wheel right direction</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetWheelRight(in RigidTransform steeredSuspensionTransform)
        {
            return math.mul(steeredSuspensionTransform.rot, math.right());
        }

        /// <summary>
        /// Gets the wheel up direction
        /// </summary>
        /// <param name="steeredSuspensionTransform"> The steered suspension transform </param>
        /// <returns>The wheel up direction</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetWheelUp(in RigidTransform steeredSuspensionTransform)
        {
            return math.mul(steeredSuspensionTransform.rot, math.up());
        }

        /// <summary>
        /// Gets the wheel forward direction
        /// </summary>
        /// <param name="steeredSuspensionTransform"> The steered suspension transform </param>
        /// <returns>The wheel forward direction</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetWheelForward(in RigidTransform steeredSuspensionTransform)
        {
            return math.mul(steeredSuspensionTransform.rot, math.forward());
        }

        /// <summary>
        /// Gets the downward center point of the wheel
        /// </summary>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <param name="steeredSuspensionWorldTransform"> The steered suspension transform </param>
        /// <param name="wheelWorldTransform">The world transform of the wheel </param>
        /// <returns> The downward center point of the wheel </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetCenteredThreadPoint(in WheelOnVehicle wheelOnVehicle, in RigidTransform steeredSuspensionWorldTransform, in RigidTransform wheelWorldTransform)
        {
            return wheelWorldTransform.pos + GetSuspensionWorldDirection(in steeredSuspensionWorldTransform) * wheelOnVehicle.Wheel.Radius;
        }

        /// <summary>
        /// Returns all relevant wheel and suspension transforms information
        /// </summary>
        /// <param name="vehicleTransform">The transform of the vehicle the suspension belongs to.</param>
        /// <param name="wheelOnVehicle">The target wheel.</param>
        /// <param name="steeredSuspensionWorldTransform"> The steered suspension transform </param>
        /// <param name="wheelWorldTransform"> The world transform of the wheel </param>
        /// <param name="suspensionDirection"> The world suspension direction </param>
        /// <param name="centeredThreadPoint"> The centered thread point on the wheel </param>
        /// <param name="wheelForward"> The wheel forward direction </param>
        /// <param name="wheelRight"> The wheel right direction </param>
        /// <param name="wheelUp"> The wheel up direction </param>
        public static void GetWheelTransforms(
            in RigidTransform vehicleTransform,
            in WheelOnVehicle wheelOnVehicle,
            out RigidTransform steeredSuspensionWorldTransform,
            out RigidTransform wheelWorldTransform,
            out float3 suspensionDirection,
            out float3 centeredThreadPoint,
            out float3 wheelForward,
            out float3 wheelRight,
            out float3 wheelUp)
        {
            steeredSuspensionWorldTransform = GetSteeredSuspensionWorldTransform(in vehicleTransform, in wheelOnVehicle);
            wheelWorldTransform = GetWheelWorldTransform(in steeredSuspensionWorldTransform, wheelOnVehicle.SuspensionLength);
            suspensionDirection = GetSuspensionWorldDirection(in steeredSuspensionWorldTransform);
            centeredThreadPoint = VehicleUtilities.GetCenteredThreadPoint(in wheelOnVehicle, in steeredSuspensionWorldTransform,
                in wheelWorldTransform);
            wheelForward = VehicleUtilities.GetWheelForward(in steeredSuspensionWorldTransform);
            wheelUp = VehicleUtilities.GetWheelUp(in steeredSuspensionWorldTransform);
            wheelRight = VehicleUtilities.GetWheelRight(in steeredSuspensionWorldTransform);
        }
    
        internal static void UpdateWheelTransforms(
            float deltaTime,
            in RigidTransform vehicleTransform, 
            ref DynamicBuffer<WheelOnVehicle> vehicleWheelsBuffer,
            ref ComponentLookup<LocalTransform> localTransformLookup)
        {
            for (int i = 0; i < vehicleWheelsBuffer.Length; i++)
            {
                WheelOnVehicle wheelOnVehicle = vehicleWheelsBuffer[i];
                if (localTransformLookup.HasComponent(wheelOnVehicle.Entity))
                {
                    // Calculate new wheel position relative to the parent vehicleTransform but do not store the values
                    // as that is reserved for the physics step.
                    RigidTransform newSuspensionWorldTransform =
                        VehicleUtilities.GetSteeredSuspensionWorldTransform(in vehicleTransform, in wheelOnVehicle);

                    // SuspensionLength is the same between the physics updates, so update the wheel position just 
                    // from the new vehicle and suspension transforms.
                    wheelOnVehicle.VisualSuspensionLength = math.lerp(wheelOnVehicle.VisualSuspensionLength,
                        wheelOnVehicle.SuspensionLength,
                        MathUtilities.GetSharpnessInterpolant(wheelOnVehicle.Wheel.VisualSuspensionSharpness, deltaTime));

                    RigidTransform newWheelWorldTransform =
                        VehicleUtilities.GetWheelWorldTransform(in newSuspensionWorldTransform,
                            wheelOnVehicle.VisualSuspensionLength);

                    // Update the visual wheel position
                    localTransformLookup[wheelOnVehicle.Entity] =
                        LocalTransform.FromPositionRotation(newWheelWorldTransform.pos, newWheelWorldTransform.rot);

                    vehicleWheelsBuffer[i] = wheelOnVehicle;
                }
            }
        }
    }
}