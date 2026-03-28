using BepuWrapper.Entities.Behaviours;
using BepuWrapper.patches;
using System;
using System.Collections.Generic;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace BepuWrapper.Api.CollisionSource
{
    public class BepuDynamicCollisionSource : IBepuDynamicCollisionSource
    {
        public void CollectCollisionBoxes(
            Entity movingEntity,
            int dimension,
            Cuboidd queryBox,
            List<DynamicCollisionBox> results)
        {
            double centerX = (queryBox.X1 + queryBox.X2) * 0.5;
            double centerY = (queryBox.Y1 + queryBox.Y2) * 0.5;
            double centerZ = (queryBox.Z1 + queryBox.Z2) * 0.5;

            double rangeX = Math.Max(1.0, (queryBox.X2 - queryBox.X1) * 0.5 + 2.0);
            double rangeY = Math.Max(1.0, (queryBox.Y2 - queryBox.Y1) * 0.5 + 2.0);

            Entity[] nearby = movingEntity.World.GetEntitiesAround(
                new Vec3d(centerX, centerY, centerZ),
                (float)rangeX,
                (float)rangeY,
                e => e != null && e.EntityId != movingEntity.EntityId
            );

            if (nearby == null || nearby.Length == 0)
                return;

            for (int i = 0; i < nearby.Length; i++)
            {
                Entity candidate = nearby[i];
                if (candidate == null || candidate.EntityId == movingEntity.EntityId)
                    continue;

                BepuPhysicsBehaviour bepuBehavior = candidate.GetBehavior<BepuPhysicsBehaviour>();
                if (bepuBehavior == null)
                    continue;

                bepuBehavior.AppendDynamicCollisionBoxes(queryBox, results);
            }
        }
    }
}
