using System.Collections.Generic;
using System.Numerics;

namespace BepuWrapper.Api
{
    public struct BuiltCompound
    {
        public Vector3 LocalCenterOfMassOffset;
        public List<ManualChildBox> ManualChildBoxes;
    }
}