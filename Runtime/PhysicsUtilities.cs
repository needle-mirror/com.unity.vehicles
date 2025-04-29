using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Authoring;

namespace Unity.Vehicles
{
    /// <summary>
    /// Various physics utilities
    /// </summary>
    public static class PhysicsUtilities
    {
        /// <summary>
        /// Determines if the specified physics body has a given physics body tag
        /// </summary>
        /// <param name="physicsWorld"> The Physics World of the body </param>
        /// <param name="bodyIndex"> The body index </param>
        /// <param name="tag"> The physics body tag to check for </param>
        /// <returns> True if the body has the physics tag </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool HasPhysicsTag(in PhysicsWorld physicsWorld, int bodyIndex, CustomPhysicsBodyTags tag)
        {
            if (tag.Value > CustomPhysicsBodyTags.Nothing.Value)
            {
                if ((physicsWorld.Bodies[bodyIndex].CustomTags & tag.Value) > 0)
                {
                    return true;
                }
            }
            return false;
        }

        
        /// <summary>
        /// Sets the collision response of a physics collider, including all child colliders if it's a compound collider.
        /// Note that if the collider is not made unique, this will affect all instances of the collider.
        /// </summary>
        /// <param name="physicsCollider">The physics collider to change</param>
        /// <param name="policy">The collision response policy to apply</param>
        public static unsafe void SetCollisionResponseAll(ref PhysicsCollider physicsCollider, CollisionResponsePolicy policy)
        {
            physicsCollider.ColliderPtr->SetCollisionResponse(policy);
            if (physicsCollider.ColliderPtr->Type == ColliderType.Compound)
            {
                CompoundCollider* compoundColliderPtr = (CompoundCollider*)physicsCollider.ColliderPtr;
                for (int j = 0; j < compoundColliderPtr->NumChildren; j++)
                {
                    ColliderKey childColliderKey = compoundColliderPtr->ConvertChildIndexToColliderKey(j);
                    physicsCollider.ColliderPtr->SetCollisionResponse(policy, childColliderKey);
                }
            }
        }
        
        /// <summary>
        /// Sets the collision filter of a physics collider, including all child colliders if it's a compound collider.
        /// Note that if the collider is not made unique, this will affect all instances of the collider.
        /// </summary>
        /// <param name="physicsCollider">The physics collider to change</param>
        /// <param name="filter">The collision filter to apply</param>
        public static unsafe void SetCollisionFilterAll(ref PhysicsCollider physicsCollider, CollisionFilter filter)
        {
            physicsCollider.ColliderPtr->SetCollisionFilter(filter);
            if (physicsCollider.ColliderPtr->Type == ColliderType.Compound)
            {
                CompoundCollider* compoundColliderPtr = (CompoundCollider*)physicsCollider.ColliderPtr;
                for (int j = 0; j < compoundColliderPtr->NumChildren; j++)
                {
                    ColliderKey childColliderKey = compoundColliderPtr->ConvertChildIndexToColliderKey(j);
                    physicsCollider.ColliderPtr->SetCollisionFilter(filter, childColliderKey);
                }
            }
        }
    }
}