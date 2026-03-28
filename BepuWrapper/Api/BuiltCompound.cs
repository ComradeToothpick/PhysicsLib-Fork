using System.Collections.Generic;
using System.Numerics;

namespace BepuWrapper.Api
{
    public struct BuiltCompound
    {
        public Compound Compound;
        public Vector3 LocalCenterOfMassOffset;
        public List<ManualChildBox> ManualChildBoxes;
        public float BroadphaseRadius;
    }
}