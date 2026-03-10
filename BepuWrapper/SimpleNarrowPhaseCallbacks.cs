using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace BepuWrapper
{
    public struct SimpleNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation) { }

        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            => true;

        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            => true;

        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            //The IContactManifold parameter includes functions for accessing contact data regardless of what the underlying type of the manifold is.
            //If you want to have direct access to the underlying type, you can use the manifold.Convex property and a cast like Unsafe.As<TManifold, ConvexContactManifold or NonconvexContactManifold>(ref manifold).

            //The engine does not define any per-body material properties. Instead, all material lookup and blending operations are handled by the callbacks.
            //For the purposes of this demo, we'll use the same settings for all pairs.
            //(Note that there's no 'bounciness' or 'coefficient of restitution' property!
            //Bounciness is handled through the contact spring settings instead. Setting See here for more details: https://github.com/bepu/bepuphysics2/issues/3 and check out the BouncinessDemo for some options.)
            pairMaterial.FrictionCoefficient = 1f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(8, 0.2f);
            return true;
        }

        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }

        public void Dispose() { }
    }
}
