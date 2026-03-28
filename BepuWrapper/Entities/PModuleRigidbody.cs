using System;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace BepuWrapper.Entities
{
    internal class PModuleRigidbody : PModule
    {
        public override void Initialize(JsonObject config, Entity entity)
        {
        }

        public override bool Applicable(Entity entity, EntityPos pos, EntityControls controls)
        {
            return true;
        }

        public override void DoApply(float dt, Entity entity, EntityPos pos, EntityControls controls)
        {
            if (entity.Api.Side == EnumAppSide.Client) 
            {
                Console.WriteLine("dooting client");
            } else
            {
                Console.WriteLine("dooting server");
            }
            if (entity.WatchedAttributes.GetInt("physcoll") == 1)
            {
                double motionX = entity.WatchedAttributes.GetDouble("rbodirX");
                double motionY = entity.WatchedAttributes.GetDouble("rbodirY");
                double motionZ = entity.WatchedAttributes.GetDouble("rbodirZ");
                double offsetX = entity.WatchedAttributes.GetDouble("offX");
                double offsetY = entity.WatchedAttributes.GetDouble("offY");
                double offsetZ = entity.WatchedAttributes.GetDouble("offZ");
                bool pushedUp = entity.WatchedAttributes.GetBool("pushedUp");
                Vec3d motion = new Vec3d(motionX, motionY, motionZ);
                pos.Motion.Set(motion);

                //pos.SetPos(pos.X + offsetX, pos.Y + offsetY, pos.Z + offsetZ);
                
                entity.OnGround = true;

                entity.WatchedAttributes.SetInt("physcoll", 0);


            }
        }
    }
}
