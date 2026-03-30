using BepuWrapper.Entities.Behaviours;
using Vintagestory.API.Client;

namespace BepuWrapper.Client
{
    public class DebugRenderer : IRenderer
    {
        private readonly BepuPhysicsBehaviour behavior;

        public double RenderOrder => 1;
        public int RenderRange => 9999;

        public DebugRenderer(BepuPhysicsBehaviour behavior)
        {
            this.behavior = behavior;
        }

        public void OnRenderFrame(float deltaTime, EnumRenderStage stage)
        {
            if (behavior.entity?.Alive == true)
            {
                behavior.DebugRender((behavior.entity.Api as ICoreClientAPI)!);
            }
        }

        public void Dispose() { }
    }
}
