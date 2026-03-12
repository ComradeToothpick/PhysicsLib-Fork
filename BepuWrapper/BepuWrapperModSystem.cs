using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using BepuWrapper.Entities.Behaviours;
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

        // Called on server and client
        // Useful for registering block/entity classes on both sides
        public override void Start(ICoreAPI api)
        {
            var harmony = new Harmony(Mod.Info.ModID);
            harmony.PatchAll();
            Mod.Logger.Notification("Hello from template mod: " + api.Side);
            api.RegisterEntityBehaviorClass("bepu-physics", typeof(BepuPhysicsBehaviour));
        }

        public override void StartServerSide(ICoreServerAPI api)
        {
            bepu = new BepuWorld(api);
            Mod.Logger.Notification("Hello from template mod server side: " + Lang.Get("bepuwrapper:hello"));
        }

        public override void StartClientSide(ICoreClientAPI api)
        {
            Mod.Logger.Notification("Hello from template mod client side: " + Lang.Get("bepuwrapper:hello"));
        }
    }
}
