using System.Collections.Generic;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace PhysicsLib.Api.CollisionSource
{
    public interface IDynamicCollisionSource
    {
        void CollectCollisionBoxes(Entity movingEntity, Cuboidd queryBox, List<DynamicCollisionBox> results);
    }
}
