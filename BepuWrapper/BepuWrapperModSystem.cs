using BepuWrapper.Api.CollisionSource;
using BepuWrapper.Entities.Behaviours;
using BepuWrapper.patches;
using HarmonyLib;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Config;
using Vintagestory.API.Server;

namespace BepuWrapper
{
    public class BepuWrapperModSystem : ModSystem
    {
        public BepuWorld bepu;
        private Harmony harmony;

        // Called on server and client
        // Useful for registering block/entity classes on both sides
        public override void Start(ICoreAPI api)
        {
            harmony = new Harmony(Mod.Info.ModID);
            harmony.PatchAll();
            bepu = new BepuWorld(api);
            CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource = new BepuDynamicCollisionSource();
            Mod.Logger.Notification("Hello from template mod: " + api.Side);
            api.RegisterEntityBehaviorClass("bepu-physics", typeof(BepuPhysicsBehaviour));
        }

        public override void StartServerSide(ICoreServerAPI api)
        {
            Mod.Logger.Notification("Hello from template mod server side: " + Lang.Get("bepuwrapper:hello"));
        }

        public override void StartClientSide(ICoreClientAPI api)
        {
            Mod.Logger.Notification("Hello from template mod client side: " + Lang.Get("bepuwrapper:hello"));
        }

        public override void Dispose()
        {
            CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource = null;

            if (harmony != null)
            {
                harmony.UnpatchAll(harmony.Id);
                harmony = null;
            }

            base.Dispose();
        }
    }
}
