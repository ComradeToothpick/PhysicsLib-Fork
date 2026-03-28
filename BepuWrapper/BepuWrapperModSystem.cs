using BepuWrapper.Api;
using BepuWrapper.Api.CollisionSource;
using BepuWrapper.Entities.Behaviours;
using BepuWrapper.patches;
using HarmonyLib;
using System.Collections.Generic;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Config;
using Vintagestory.API.Server;
using Vintagestory.API.Util;

namespace BepuWrapper
{
    public class BepuWrapperModSystem : ModSystem
    {
        private Harmony harmony;
        private Dictionary<string, CachedCompound> ComputedShapes = new Dictionary<string, CachedCompound>();

        // Called on server and client
        // Useful for registering block/entity classes on both sides
        public override void Start(ICoreAPI api)
        {
            harmony = new Harmony(Mod.Info.ModID);
            harmony.PatchAll();
            var dynamicSource = new BepuDynamicCollisionSource(api);
            CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource = dynamicSource;
            CollisionTester_IsColliding_Patch.DynamicCollisionSource = dynamicSource;
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


        public CachedCompound AddCompoundShape(string shapeCode, BuiltCompound shape)
        {
            var cacheItem = new CachedCompound()
            {
                BroadphaseRadius = shape.BroadphaseRadius,
                Inertia = shape.Inertia,
                LocalCenterOfMassOffset = shape.LocalCenterOfMassOffset,
                ManualChildBoxes = shape.ManualChildBoxes
            };

            ComputedShapes.TryAdd(shapeCode, cacheItem);

            return cacheItem;
        }

        public CachedCompound? TryGetCompoundShape(string shapeCode)
        {
            return ComputedShapes.ContainsKey(shapeCode) ? ComputedShapes.Get(shapeCode) : null;
        }

        public override void Dispose()
        {
            CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource = null;
            CollisionTester_IsColliding_Patch.DynamicCollisionSource = null;

            if (harmony != null)
            {
                harmony.UnpatchAll(harmony.Id);
                harmony = null;
            }

            base.Dispose();
        }
    }
}
