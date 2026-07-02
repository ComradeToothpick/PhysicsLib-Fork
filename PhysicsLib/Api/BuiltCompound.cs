using System;
using System.Collections.Generic;
using System.Numerics;

namespace PhysicsLib.Api
{
    public class BuiltCompound
    {
        public Vector3 LocalCenterOfMassOffset;
        public List<LocalBox> Boxes;
    }
}