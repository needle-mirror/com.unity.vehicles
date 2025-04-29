# Netcode

If vehicles must be networked and predicted, and the vehicle is using `VehicleControlAuthoring` and `ControllableWheelAuthoring`, the following steps are required to set things up correctly:

## VehicleControlPredictionSystem
A new `VehicleControlPredictionSystem` should be created, in order to schedule vehicle control jobs in prediction:
```cs
using Unity.Burst;
using Unity.Entities;
using Unity.NetCode;
using Unity.Physics;
using Unity.Vehicles;

[BurstCompile]
[UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderFirst = true)]
[UpdateBefore(typeof(PredictedFixedStepSimulationSystemGroup))]
[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.ServerSimulation)]
public partial struct VehicleControlPredictionSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<PhysicsWorldSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        VehicleControlSystem.VehicleControlJob job = new VehicleControlSystem.VehicleControlJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            PhysicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld,
            WheelControlLookup = SystemAPI.GetComponentLookup<WheelControl>(true),
            EngineControlLookup = SystemAPI.GetComponentLookup<EngineStartStop>(false),
            VehicleUpdateCollidersLookup = SystemAPI.GetComponentLookup<VehicleUpdateColliders>(false)
        };
        state.Dependency = job.ScheduleParallel(state.Dependency);

        state.Dependency = new VehicleControlSystem.VehicleEngineControlJob
        {
            EngineControlLookup = SystemAPI.GetComponentLookup<EngineStartStop>(false),
        }.Schedule(state.Dependency);
    }
}
```

## Predicted User Vehicle Control
The system responsible for writing intputs to the `VehicleControl` component should now have these update rules:
* `[UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderFirst = true)]`
* `[UpdateBefore(typeof(PredictedFixedStepSimulationSystemGroup))]`
* `[UpdateAfter(typeof(CopyCommandBufferToInputSystemGroup))]`
* `[UpdateBefore(typeof(VehicleControlPredictionSystem))]`


## Ghost Variants
When using `VehicleControlAuthoring`, the following ghost variants are necessary to network the vehicle properly for prediction support:

```cs
[GhostComponentVariation(typeof(VehicleControlData))]
public struct VehicleControlData_GhostVariant
{
    [GhostField()]
    public float SteeringPosition;
    [GhostField()]
    public int TransmissionCurrentGear;
    [GhostField()]
    public float TransmissionCurrentGearRatio;
    [GhostField()]
    public float EngineAngularVelocity;
    [GhostField()]
    public bool EngineIsRunning;
}

[GhostComponentVariation(typeof(WheelOnVehicle))]
public struct WheelOnVehicle_GhostVariant
{
    [GhostField(SendData = false)]
    public int WheelProtectorChildColliderIndex;
    [GhostField(SendData = false)]
    public RigidTransform SuspensionLocalTransform;
    [GhostField(SendData = false)]
    public Entity AxlePairedWheelEntity;
    [GhostField(SendData = false)]
    public int AxlePairedWheelIndex;
    [GhostField(SendData = false)]
    public Entity Entity;
    [GhostField(SendData = false)]
    public Wheel Wheel;
    [GhostField(SendData = false)]
    public float MotorTorque;
    [GhostField(SendData = false)]
    public float BrakeTorque;
    [GhostField(SendData = false)]
    public float AngularVelocityLimit;

    [GhostField()]
    public float SteerAngle;
    [GhostField()]
    public float AngularVelocity;
    [GhostField()]
    public float RotationAngle;
    [GhostField()]
    public float SuspensionLength;
    [GhostField(SendData = false)]
    public float VisualSuspensionLength;

    [GhostField(SendData = false)]
    public bool IsGrounded;
    [GhostField(SendData = false)]
    public ColliderCastHit WheelHit;
    [GhostField(SendData = false)]
    public float3 SuspensionImpulse;
    [GhostField(SendData = false)]
    public float3 FrictionImpulse;
    [GhostField()]
    public float2 FrictionSlip;
    [GhostField(SendData = false)]
    public float2 FrictionSpeed;

    [GhostField()]
    public bool StaticFrictionReferenceIsSet;
    [GhostField()]
    public float3 StaticFrictionRefPosition;
    [GhostField(SendData = false)]
    public bool DisableStaticFrictionSingleFrame;
}
```