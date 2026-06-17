using PhysicsLib.Entities.Behaviours;
using Vintagestory.API.Client;

namespace PhysicsLib.Client
{
    public class DebugRenderer : IRenderer
    {
        private readonly DynamicPhysicsBehaviour behavior;

        public double RenderOrder => 1;
        public int RenderRange => 9999;

        public DebugRenderer(DynamicPhysicsBehaviour behavior)
        {
            this.behavior = behavior;
        }

        public void OnRenderFrame(float deltaTime, EnumRenderStage stage)
        {
            if (behavior.entity?.Alive == true)
            {
                //behavior.DebugRender((behavior.entity.Api as ICoreClientAPI)!);
            }
        }

        public void Dispose() { }
    }
}
