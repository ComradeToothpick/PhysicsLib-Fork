using System.Collections.Generic;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace BepuWrapper.Api.CollisionSource
{
    public interface IBepuDynamicCollisionSource
    {
        void CollectCollisionBoxes(Entity movingEntity, int dimension, Cuboidd queryBox, List<DynamicCollisionBox> results);
    }
}
