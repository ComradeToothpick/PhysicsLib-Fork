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

        public DynamicCollisionSource(ICoreAPI api) { this.api = api; }

        public void CollectCollisionBoxes(
            Entity movingEntity,
            Cuboidd queryBox,
            List<DynamicCollisionBox> results)
        {
            double centerX = (queryBox.X1 + queryBox.X2) * 0.5;
            double centerY = (queryBox.Y1 + queryBox.Y2) * 0.5;
            double centerZ = (queryBox.Z1 + queryBox.Z2) * 0.5;

            double rangeX = Math.Max(1.0, (queryBox.X2 - queryBox.X1) * 0.5 + 2.0);
            double rangeY = Math.Max(1.0, (queryBox.Y2 - queryBox.Y1) * 0.5 + 2.0);
            Entity[] nearby;
            if (movingEntity != null)
                nearby = movingEntity.World.GetEntitiesAround(
                    new Vec3d(centerX, centerY, centerZ),
                    (float)rangeX,
                    (float)rangeY,
                    e => e != null && e.EntityId != movingEntity.EntityId
                );
            else 
                 nearby = api.World.GetEntitiesAround(
                    new Vec3d(centerX, centerY, centerZ),
                    (float)rangeX,
                    (float)rangeY,
                    e => e != null
                );
            

            if (nearby == null || nearby.Length == 0)
                return;

            for (int i = 0; i < nearby.Length; i++)
            {
                Entity candidate = nearby[i];
                if (candidate == null || (movingEntity != null && candidate.EntityId == movingEntity.EntityId))
                    continue;

                DynamicPhysicsBehaviour? bepuBehavior = candidate.GetBehavior<DynamicPhysicsBehaviour>();
                if (bepuBehavior == null)
                    continue;

                bepuBehavior.AppendDynamicCollisionBoxes(queryBox, results);
            }
        }
    }
}
