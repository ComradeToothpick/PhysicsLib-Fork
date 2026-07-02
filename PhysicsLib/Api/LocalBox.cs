using System.Numerics;

namespace PhysicsLib.Api
{
    public struct LocalBox
    {
        public Vector3 LocalPosition;
        public Quaternion LocalOrientation;
        public Vector3 HalfExtents;
    }
}