using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

namespace Unity.Vehicles
{
    /// <summary>
    /// Collector for the wheel casst hits.
    /// Stores the closest hit that is not a hit to a vehicle collider.
    /// </summary>
    public struct WheelClosestHitCollector : ICollector<ColliderCastHit>
    {
        /// <summary>
        /// Set to false as there might be invalid self hits.
        /// </summary>
        public bool EarlyOutOnFirstHit => false;

        /// <summary>
        /// Maximum fraction of cast distance within which the hits will register.
        /// </summary>
        public float MaxFraction { get; private set; }

        /// <summary>
        /// Number of registered hits.
        /// </summary>
        public int NumHits { get; private set; }

        /// <summary>
        /// Closest valid hit.
        /// </summary>
        public ColliderCastHit ClosestHit => _closestHit;

        private Entity _vehicleEntity;
        private ColliderCastHit _closestHit;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="vehicleEntity">The vehicle entity</param>
        public WheelClosestHitCollector(Entity vehicleEntity)
        {
            MaxFraction = 1f;
            NumHits = 0;

            _vehicleEntity = vehicleEntity;
            _closestHit = default;
            _closestHit.Fraction = float.MaxValue;
        }

        /// <summary>
        /// Callback on cast hit.
        /// </summary>
        /// <param name="hit">Hit info.</param>
        /// <returns>True if the hit is valid.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddHit(ColliderCastHit hit)
        {
            if (hit.Fraction < _closestHit.Fraction)
            {
                if (hit.Entity != _vehicleEntity)
                {
                    if (hit.Material.CollisionResponse == CollisionResponsePolicy.Collide ||
                        hit.Material.CollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents)
                    {
                        _closestHit = hit;
                        NumHits = 1;
                        return true;
                    }
                }
            }

            return false;
        }
    }
}