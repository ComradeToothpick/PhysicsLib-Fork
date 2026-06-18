using System.Numerics;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace PhysicsLib.Api
{
    public class DynamicCollisionBox
    {
        // Broadphase only. Never use this as the true collider.
        public Cuboidd Box = null!;

        // True collider.
        public Vector3 Center;
        public Quaternion Orientation = Quaternion.Identity;
        public Vector3 HalfExtents;

        public Entity SourceEntity = null!;
        public bool CanSupport;

        public Vec3d CenterD;
    }
}
