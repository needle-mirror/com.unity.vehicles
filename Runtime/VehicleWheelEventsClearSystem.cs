using Unity.Burst;
using Unity.Entities;

namespace Unity.Vehicles
{
    /// <summary>
    /// System that clears vehicle wheel events
    /// </summary>
    [UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
    public partial struct VehicleWheelEventsClearSystem : ISystem
    {
        [BurstCompile]
        internal void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<VehicleWheelEvent>();
        }

        [BurstCompile]
        internal void OnUpdate(ref SystemState state)
        {
            state.Dependency = new VehicleWheelEventsClearJob
                { }.Schedule(state.Dependency);
        }

        [BurstCompile]
        partial struct VehicleWheelEventsClearJob : IJobEntity
        {
            void Execute(ref DynamicBuffer<VehicleWheelEvent> vehicleEventsBuffer)
            {
                vehicleEventsBuffer.Clear();
            }
        }
    }
}