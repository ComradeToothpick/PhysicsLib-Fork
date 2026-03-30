using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace PhysicsLib.Api
{
    public struct DynamicCollisionBox
    {
        public Cuboidd Box;
        public Entity SourceEntity;
        public bool CanSupport;
    }
}
