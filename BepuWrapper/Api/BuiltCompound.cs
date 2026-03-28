using BepuPhysics;
using BepuPhysics.Collidables;
using System.Collections.Generic;
using System.Numerics;

namespace BepuWrapper.Api
{
    public struct BuiltCompound
    {
        public Compound Compound;
        public BodyInertia Inertia;
        public Vector3 LocalCenterOfMassOffset;
        public List<ManualChildBox> ManualChildBoxes;
        public float BroadphaseRadius;
    }
}