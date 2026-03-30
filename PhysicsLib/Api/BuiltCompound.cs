using System.Collections.Generic;
using System.Numerics;

namespace PhysicsLib.Api
{
    public struct BuiltCompound
    {
        public Vector3 LocalCenterOfMassOffset;
        public List<ManualChildBox> ManualChildBoxes;
    }
}