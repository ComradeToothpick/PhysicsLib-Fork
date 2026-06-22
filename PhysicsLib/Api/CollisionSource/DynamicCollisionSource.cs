using PhysicsLib.Entities.Behaviours;
using System;
using System.Collections.Generic;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace PhysicsLib.Api.CollisionSource
{
    public class DynamicCollisionSource : IDynamicCollisionSource
    {
        private ICoreAPI api;

        // All registered physics entities. Registered on behaviour Initialize,
        // unregistered on despawn. We iterate this directly rather than using
        // GetEntitiesAround, which uses the entity's vanilla CollisionBox for
        // spatial lookup — that box only covers the entity origin, so the rear
        // of a long boat (many blocks from origin) is never found.
        private readonly List<DynamicPhysicsBehaviour> registeredBehaviours = new();

        public DynamicCollisionSource(ICoreAPI api) { this.api = api; }

        public void Register(DynamicPhysicsBehaviour behaviour)
        {
            if (!registeredBehaviours.Contains(behaviour))
                registeredBehaviours.Add(behaviour);
        }

        public void Unregister(DynamicPhysicsBehaviour behaviour)
        {
            registeredBehaviours.Remove(behaviour);
        }

        public void CollectCollisionBoxes(
            Entity movingEntity,
            Cuboidd queryBox,
            ref List<DynamicCollisionBox> results)
        {
            for (int i = registeredBehaviours.Count - 1; i >= 0; i--)
            {
                DynamicPhysicsBehaviour behaviour = registeredBehaviours[i];

                if (behaviour == null || behaviour.entity == null || !behaviour.entity.Alive)
                {
                    registeredBehaviours.RemoveAt(i);
                    continue;
                }

                if (movingEntity != null && behaviour.entity.EntityId == movingEntity.EntityId)
                    continue;

                behaviour.AppendDynamicCollisionBoxes(queryBox, ref results);
            }
        }
    }
}