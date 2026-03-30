using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using PhysicsLib.Entities.Behaviours;
using PhysicsLib.patches;
using HarmonyLib;
using System.Collections.Generic;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Config;
using Vintagestory.API.Server;
using Vintagestory.API.Util;

namespace PhysicsLib
{
    public class PhysicsLibModSystem : ModSystem
    {
        private Harmony? harmony;
        private Dictionary<string, BuiltCompound> ComputedShapes = new Dictionary<string, BuiltCompound>();

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
            Mod.Logger.Notification("Hello from template mod server side: " + Lang.Get("physicslib:hello"));
        }

        public override void StartClientSide(ICoreClientAPI api)
        {
            Mod.Logger.Notification("Hello from template mod client side: " + Lang.Get("physicslib:hello"));
        }


        public BuiltCompound AddCompoundShape(string shapeCode, BuiltCompound shape)
        {
            ComputedShapes.TryAdd(shapeCode, shape);

            return shape;
        }

        public BuiltCompound? TryGetCompoundShape(string shapeCode)
        {
            return ComputedShapes.ContainsKey(shapeCode) ? ComputedShapes.Get(shapeCode) : null;
        }

        public override void Dispose()
        {
            CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource = null!;
            CollisionTester_IsColliding_Patch.DynamicCollisionSource = null!;

            if (harmony != null)
            {
                harmony.UnpatchAll(harmony.Id);
                harmony = null!;
            }

            base.Dispose();
        }
    }
}
