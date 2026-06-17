using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using PhysicsLib.Entities.Behaviours;
using HarmonyLib;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace PhysicsLib.patches
{
    [HarmonyPatch(typeof(CollisionTester), nameof(CollisionTester.ApplyTerrainCollision))]
    public static class CollisionTester_ApplyTerrainCollision_Patch
    {
        public static IDynamicCollisionSource? DynamicCollisionSource;

        // ── support tracking ──────────────────────────────────────────────────────
        private sealed class SupportState
        {
            public long SupportEntityId;
            public Vector3 LocalAnchorPoint;
        }

        private sealed class SupportCandidate
        {
            public Entity SupportEntity = null!;
            public DynamicCollisionBox DynamicBox = null!;
            public double TopY;
        }

        private static readonly Dictionary<long, SupportState> SupportStates = new();

        // ── tuning constants ──────────────────────────────────────────────────────

        // Small directional bias so the entity is always nudging away from surfaces
        // rather than sitting exactly flush (prevents jitter on the sweep boundary).
        private const double MotionBiasThreshold = 0.0001;

        // How far above/below the surface top the feet may be and still be considered
        // "standing on" a dynamic collider.  Above = feet slightly in the air (step up,
        // bobbing boat).  Below = feet barely inside the surface (resting contact).
        private const double SupportProbeAbove = 0.05;   // scan this far above current feet
        private const double SupportProbeBelow = 0.12;   // scan this far below current feet

        // Surface must face at least this far upward to be a floor candidate.
        private const double MinSupportUpY = 0.65;

        // Foot-sample inset so corner samples stay inside a narrow deck plank.
        private const double SupportFootInset = 0.025;

        // Extra XZ padding when checking whether foot samples are over a collider.
        private const double SupportHorizontalPadding = 0.015;

        // A tiny gap left between the entity and surfaces after a sweep so we never
        // land exactly flush (avoids re-triggering the sweep the next frame).
        private const double SweepSkin = 0.001;

        // ── harmony entry point ───────────────────────────────────────────────────

        [HarmonyPrefix]
        public static bool Prefix(
            CollisionTester __instance,
            Entity entity,
            EntityPos entityPos,
            float dtFactor,
            ref Vec3d newPosition,
            float stepHeight = 1f,
            float yExtra = 1f)
        {
            ApplyTerrainAndDynamicCollision(
                __instance, entity, entityPos, dtFactor,
                ref newPosition, stepHeight, yExtra);
            return false;
        }

        // ── public support helpers ────────────────────────────────────────────────

        public static void ClearStandingOnEntity(Entity entity)
        {
            if (entity == null) return;
            SupportStates.Remove(entity.EntityId);
        }

        // ── private support state ─────────────────────────────────────────────────

        private static void SetStandingOnEntity(Entity entity, Entity supportEntity, Vector3 localAnchorPoint)
        {
            if (entity == null || supportEntity == null) return;
            SupportStates[entity.EntityId] = new SupportState
            {
                SupportEntityId = supportEntity.EntityId,
                LocalAnchorPoint = localAnchorPoint
            };
        }

        private static Entity? ResolveSupportEntity(Entity entity, out SupportState? supportState)
        {
            supportState = null;
            if (entity == null) return null;
            if (!SupportStates.TryGetValue(entity.EntityId, out supportState) || supportState == null)
                return null;
            return entity.World.GetEntityById(supportState.SupportEntityId);
        }

        // ── main collision routine ────────────────────────────────────────────────

        private static void ApplyTerrainAndDynamicCollision(
            CollisionTester tester,
            Entity entity,
            EntityPos entityPos,
            float dtFactor,
            ref Vec3d newPosition,
            float stepHeight,
            float yExtra)
        {
            tester.minPos.dimension = entityPos.Dimension;

            IWorldAccessor world = entity.World;
            Vec3d pos = tester.pos;
            Cuboidd entityBox = tester.entityBox;
            BlockPos collBlockPos = new BlockPos(entityPos.Dimension);

            pos.X = entityPos.X;
            pos.Y = entityPos.Y;
            pos.Z = entityPos.Z;

            entityBox.SetAndTranslate(entity.CollisionBox, pos.X, pos.Y, pos.Z);

            // ── carry delta from standing on a moving entity ──────────────────────
            // We apply the support platform's movement to this frame's motion so the
            // player moves with the boat without a one-frame lag.
            Entity? prevSupport = ResolveSupportEntity(entity, out SupportState? prevSupportState);
            DynamicPhysicsBehaviour? prevSupportPhysics = prevSupport?.GetBehavior<DynamicPhysicsBehaviour>();

            Vec3d carryDelta = Vec3d.Zero;
            if (prevSupport != null && prevSupportPhysics != null && prevSupportState != null)
                prevSupportPhysics.TryGetPointVelocityDelta(prevSupportState.LocalAnchorPoint, out carryDelta);

            double motionX = entityPos.Motion.X * dtFactor + carryDelta.X;
            double motionY = entityPos.Motion.Y * dtFactor + carryDelta.Y;
            double motionZ = entityPos.Motion.Z * dtFactor + carryDelta.Z;

            // Small bias so we never sit exactly on a sweep boundary.
            double biasX = motionX > MotionBiasThreshold ? MotionBiasThreshold :
                           motionX < -MotionBiasThreshold ? -MotionBiasThreshold : 0.0;
            double biasY = motionY > MotionBiasThreshold ? MotionBiasThreshold :
                           motionY < -MotionBiasThreshold ? -MotionBiasThreshold : 0.0;
            double biasZ = motionZ > MotionBiasThreshold ? MotionBiasThreshold :
                           motionZ < -MotionBiasThreshold ? -MotionBiasThreshold : 0.0;

            motionX += biasX;
            motionY += biasY;
            motionZ += biasZ;

            // ── collect collision geometry ────────────────────────────────────────
            GenerateTerrainCollisionBoxList(
                tester, world.BlockAccessor,
                motionX, motionY, motionZ,
                stepHeight, yExtra, entityPos.Dimension);

            List<DynamicCollisionBox> dynBoxes = CollectDynamicCollisionBoxes(
                entity, entityPos.Dimension, entityBox,
                motionX, motionY, motionZ, stepHeight, yExtra);

            int terrainCount = tester.CollisionBoxList.Count;
            Cuboidd[] terrainBoxes = tester.CollisionBoxList.cuboids;

            bool collidedV = false;
            bool collidedH = false;

            // ════════════════════════════════════════════════════════════════════════
            // Y PASS — resolve vertical motion against terrain then dynamic colliders
            // ════════════════════════════════════════════════════════════════════════

            for (int i = 0; i < terrainCount && i < terrainBoxes.Length; i++)
            {
                EnumPushDirection dir = EnumPushDirection.None;
                motionY = terrainBoxes[i].pushOutY(entityBox, motionY, ref dir);
                if (dir == EnumPushDirection.None) continue;

                collidedV = true;
                collBlockPos.Set(tester.CollisionBoxList.positions[i]);
                tester.CollisionBoxList.blocks[i].OnEntityCollide(
                    world, entity, collBlockPos,
                    dir == EnumPushDirection.Negative ? BlockFacing.UP : BlockFacing.DOWN,
                    tester.tmpPosDelta.Set(motionX, motionY, motionZ),
                    !entity.CollidedVertically);
            }

            for (int i = 0; i < dynBoxes.Count; i++)
            {
                EnumPushDirection dir = EnumPushDirection.None;
                double swept = SweepAxisObbAabb(dynBoxes[i], entityBox, motionY, 1, ref dir);
                if (dir == EnumPushDirection.None) continue;
                motionY = swept;
                collidedV = true;
            }

            // Commit Y so the horizontal passes use the correct foot position.
            entityBox.Translate(0.0, motionY, 0.0);
            entity.CollidedVertically = collidedV;

            // ════════════════════════════════════════════════════════════════════════
            // SUPPORT SCAN — completely independent of the collision passes above.
            //
            // We do NOT derive "standing on" from collision results.  Instead we cast
            // a thin foot probe downward from pos.Y (the entity's origin, not Y1 of
            // whichever collision box is current) to find any dynamic surface close
            // beneath the feet.  This is stance-independent: crouching changes Y2 but
            // pos.Y / feet-bottom is always the entity's world origin.
            // ════════════════════════════════════════════════════════════════════════

            // Foot position after Y motion has been resolved.
            double feetY = entityPos.Y + motionY - biasY;

            // Probe box: a thin slab centred on the XZ position of the entity,
            // spanning [feetY - SupportProbeBelow .. feetY + SupportProbeAbove].
            // Width matches the entity's X/Z footprint; height is irrelevant to
            // TryGetSupportTopUnderFeet which works from .Y1 (the feet level).
            double probeX1 = entityBox.X1;
            double probeX2 = entityBox.X2;
            double probeZ1 = entityBox.Z1;
            double probeZ2 = entityBox.Z2;
            Cuboidd probeBox = new Cuboidd(
                probeX1, feetY - SupportProbeBelow, probeZ1,
                probeX2, feetY + SupportProbeAbove, probeZ2);

            // Collect dynamic boxes that overlap the probe region (zero motion query).
            List<DynamicCollisionBox> supportBoxes = CollectDynamicCollisionBoxes(
                entity, entityPos.Dimension, probeBox,
                0.0, 0.0, 0.0, stepHeight, yExtra);

            // Find the highest surface whose top is within the probe band.
            // We use a probe box whose Y1 = feetY so TryGetSupportTopUnderFeet
            // samples at the right height.
            Cuboidd feetProbe = new Cuboidd(
                probeX1, feetY, probeZ1,
                probeX2, feetY + SupportProbeAbove, probeZ2);

            SupportCandidate? support = null;
            for (int i = 0; i < supportBoxes.Count; i++)
            {
                DynamicCollisionBox dyn = supportBoxes[i];
                if (!dyn.CanSupport || dyn.SourceEntity == null) continue;

                if (!TryGetSupportTopUnderFeet(feetProbe, dyn, out double topY)) continue;

                double delta = feetY - topY;   // positive = feet above surface, negative = feet below
                if (delta < -SupportProbeAbove || delta > SupportProbeBelow) continue;

                if (support == null || topY > support.TopY)
                    support = new SupportCandidate { SupportEntity = dyn.SourceEntity, DynamicBox = dyn, TopY = topY };
            }

            // Snap feet to the surface and record support.
            if (support != null)
            {
                double snapDelta = support.TopY - feetY;
                // Only snap if we are actually on or near the surface (not mid-air above it).
                if (snapDelta >= -SupportProbeBelow && snapDelta <= SupportProbeAbove)
                {
                    motionY += snapDelta;
                    if (entityPos.Motion.Y < 0.0) entityPos.Motion.Y = 0.0;
                    entity.CollidedVertically = true;
                    entity.OnGround = true;
                }
            }

            // ════════════════════════════════════════════════════════════════════════
            // X PASS
            // ════════════════════════════════════════════════════════════════════════

            // Decide which support entity to use when ignoring its floor colliders
            // in the horizontal passes (so standing on deck doesn't push you sideways).
            Entity? resolvedSupport = support?.SupportEntity ?? prevSupport;

            for (int i = 0; i < terrainCount && i < terrainBoxes.Length; i++)
            {
                EnumPushDirection dir = EnumPushDirection.None;
                motionX = terrainBoxes[i].pushOutX(entityBox, motionX, ref dir);
                if (dir == EnumPushDirection.None) continue;

                collidedH = true;
                collBlockPos.Set(tester.CollisionBoxList.positions[i]);
                tester.CollisionBoxList.blocks[i].OnEntityCollide(
                    world, entity, collBlockPos,
                    dir == EnumPushDirection.Negative ? BlockFacing.EAST : BlockFacing.WEST,
                    tester.tmpPosDelta.Set(motionX, motionY, motionZ),
                    !entity.CollidedHorizontally);
            }

            for (int i = 0; i < dynBoxes.Count; i++)
            {
                if (IsFloorOfSupport(dynBoxes[i], resolvedSupport, entityBox)) continue;
                EnumPushDirection dir = EnumPushDirection.None;
                double swept = SweepAxisObbAabb(dynBoxes[i], entityBox, motionX, 0, ref dir);
                if (dir == EnumPushDirection.None) continue;
                motionX = swept;
                collidedH = true;
            }

            entityBox.Translate(motionX, 0.0, 0.0);

            // ════════════════════════════════════════════════════════════════════════
            // Z PASS
            // ════════════════════════════════════════════════════════════════════════

            for (int i = 0; i < terrainCount && i < terrainBoxes.Length; i++)
            {
                EnumPushDirection dir = EnumPushDirection.None;
                motionZ = terrainBoxes[i].pushOutZ(entityBox, motionZ, ref dir);
                if (dir == EnumPushDirection.None) continue;

                collidedH = true;
                collBlockPos.Set(tester.CollisionBoxList.positions[i]);
                tester.CollisionBoxList.blocks[i].OnEntityCollide(
                    world, entity, collBlockPos,
                    dir == EnumPushDirection.Negative ? BlockFacing.SOUTH : BlockFacing.NORTH,
                    tester.tmpPosDelta.Set(motionX, motionY, motionZ),
                    !entity.CollidedHorizontally);
            }

            for (int i = 0; i < dynBoxes.Count; i++)
            {
                if (IsFloorOfSupport(dynBoxes[i], resolvedSupport, entityBox)) continue;
                EnumPushDirection dir = EnumPushDirection.None;
                double swept = SweepAxisObbAabb(dynBoxes[i], entityBox, motionZ, 2, ref dir);
                if (dir == EnumPushDirection.None) continue;
                motionZ = swept;
                collidedH = true;
            }

            entity.CollidedHorizontally = collidedH;

            // ── remove bias, apply ladder fix ─────────────────────────────────────
            motionX -= biasX;
            motionY -= biasY;
            motionZ -= biasZ;

            if (motionY > 0.0 && entity.CollidedVertically)
                motionY -= entity.LadderFixDelta;

            // ── finalise position and support state ───────────────────────────────
            double finalX = pos.X + motionX;
            double finalY = pos.Y + motionY;
            double finalZ = pos.Z + motionZ;

            if (support != null)
            {
                Cuboidd finalBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);
                Vec3d feetCenter = new Vec3d(
                    (finalBox.X1 + finalBox.X2) * 0.5,
                    finalBox.Y1,
                    (finalBox.Z1 + finalBox.Z2) * 0.5);

                DynamicPhysicsBehaviour? supportPhysics =
                    support.SupportEntity.GetBehavior<DynamicPhysicsBehaviour>();

                if (supportPhysics != null &&
                    supportPhysics.TryTransformWorldPointToLocal(feetCenter, out Vector3 localAnchor))
                {
                    SetStandingOnEntity(entity, support.SupportEntity, localAnchor);
                    entity.OnGround = true;
                }
                else
                {
                    ClearStandingOnEntity(entity);
                }
            }
            else
            {
                ClearStandingOnEntity(entity);
            }

            newPosition.Set(finalX, finalY, finalZ);
        }

        // ── helper: should we skip horizontal collision against this box? ─────────
        // Returns true only when the box belongs to our support entity AND the entity's
        // feet are within the normal standing-contact band above the surface.  This
        // stops the deck's own hull colliders from pushing the player sideways.
        private static bool IsFloorOfSupport(
            DynamicCollisionBox dynBox,
            Entity? supportEntity,
            Cuboidd entityBox)
        {
            if (supportEntity == null || dynBox.SourceEntity != supportEntity) return false;
            if (!dynBox.CanSupport) return false;

            if (!TryGetSupportTopUnderFeet(entityBox, dynBox, out double topY)) return false;

            double delta = entityBox.Y1 - topY;
            // Only skip horizontal collision when the feet are within the normal
            // standing band.  If the delta is very negative the player is approaching
            // from the side and must be stopped.
            return delta >= -(SupportProbeAbove) && delta <= SupportProbeBelow;
        }

        // ── swept AABB vs OBB along one world axis ────────────────────────────────
        // This replaces the old PushOutAxisObbAabb which mixed swept and depenetration
        // logic in ways that caused teleportation.
        //
        // Contract:
        //   • If no collision: returns motion unchanged, direction = None.
        //   • If collision:    returns the largest safe motion (≥0, ≤|motion|) that
        //                      keeps a SweepSkin gap, sets direction.
        //   • Never returns a value that moves the entity in the opposite direction.
        private static double SweepAxisObbAabb(
            DynamicCollisionBox obb,
            Cuboidd entityBox,
            double motion,
            int axis,
            ref EnumPushDirection direction)
        {
            direction = EnumPushDirection.None;

            if (Math.Abs(motion) < 1e-12) return motion;

            // Broad-phase: does the swept volume even touch the OBB?
            Cuboidd movedBox = OffsetEntityBox(entityBox,
                axis == 0 ? motion : 0.0,
                axis == 1 ? motion : 0.0,
                axis == 2 ? motion : 0.0);

            bool startOverlap = IntersectsObbAabb(obb, entityBox);
            bool endOverlap = IntersectsObbAabb(obb, movedBox);

            // Already overlapping and moving away — allow, will resolve naturally.
            if (startOverlap && !endOverlap) return motion;

            // Already overlapping and moving further in — stop dead.
            // (This should be rare; ideally the swept test catches it before it happens.)
            if (startOverlap && endOverlap)
            {
                direction = motion > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;
                return 0.0;
            }

            // Not overlapping at start — run the sweep.
            if (!endOverlap) return motion;   // broad-phase miss after sweep, no collision

            Vector3 sweep = axis switch
            {
                0 => new Vector3((float)motion, 0f, 0f),
                1 => new Vector3(0f, (float)motion, 0f),
                2 => new Vector3(0f, 0f, (float)motion),
                _ => Vector3.Zero
            };

            if (!TrySweepAabbAgainstObb(entityBox, sweep, obb, out double hitFraction))
                return motion;  // sweep says no collision despite broad-phase — trust broad-phase? No — let through.

            // Leave a SweepSkin gap so we don't land flush on the surface.
            double skinFraction = Math.Abs(motion) > 1e-9 ? SweepSkin / Math.Abs(motion) : 0.0;
            double safeFraction = Math.Clamp(hitFraction - skinFraction, 0.0, 1.0);

            direction = motion > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;
            return motion * safeFraction;
        }

        // ── support surface detection ─────────────────────────────────────────────
        // Samples five foot-level points in XZ and finds the highest point on the
        // OBB surface directly below them.  Returns false if the OBB is not
        // oriented roughly upward or no sample lands over the OBB.
        private static bool TryGetSupportTopUnderFeet(
            Cuboidd entityBox,
            DynamicCollisionBox dynBox,
            out double supportTopY)
        {
            supportTopY = double.NegativeInfinity;

            // Reject heavily-tilted surfaces (e.g. a mast lying on its side).
            Vector3 upAxis = SafeNormalize(
                Vector3.Transform(Vector3.UnitY, dynBox.Orientation), Vector3.UnitY);
            if (upAxis.Y < MinSupportUpY) return false;

            Quaternion invOrientation = Quaternion.Inverse(dynBox.Orientation);

            double x1 = entityBox.X1 + SupportFootInset;
            double x2 = entityBox.X2 - SupportFootInset;
            double z1 = entityBox.Z1 + SupportFootInset;
            double z2 = entityBox.Z2 - SupportFootInset;

            // Clamp so inset never flips for narrow boxes.
            if (x1 > x2) x1 = x2 = (entityBox.X1 + entityBox.X2) * 0.5;
            if (z1 > z2) z1 = z2 = (entityBox.Z1 + entityBox.Z2) * 0.5;

            double cx = (entityBox.X1 + entityBox.X2) * 0.5;
            double cz = (entityBox.Z1 + entityBox.Z2) * 0.5;
            double fy = entityBox.Y1;   // feet level

            Span<Vector3> samples = stackalloc Vector3[5]
            {
                new Vector3((float)cx, (float)fy, (float)cz),
                new Vector3((float)x1, (float)fy, (float)z1),
                new Vector3((float)x1, (float)fy, (float)z2),
                new Vector3((float)x2, (float)fy, (float)z1),
                new Vector3((float)x2, (float)fy, (float)z2),
            };

            bool found = false;
            for (int i = 0; i < samples.Length; i++)
            {
                Vector3 local = Vector3.Transform(samples[i] - dynBox.Center, invOrientation);

                if (MathF.Abs(local.X) > dynBox.HalfExtents.X + (float)SupportHorizontalPadding) continue;
                if (MathF.Abs(local.Z) > dynBox.HalfExtents.Z + (float)SupportHorizontalPadding) continue;

                // Project sample onto the top face of the OBB.
                Vector3 topLocal = new Vector3(local.X, dynBox.HalfExtents.Y, local.Z);
                Vector3 topWorld = dynBox.Center + Vector3.Transform(topLocal, dynBox.Orientation);

                if (!found || topWorld.Y > supportTopY)
                {
                    supportTopY = topWorld.Y;
                    found = true;
                }
            }

            return found;
        }

        // ── terrain collision box list ────────────────────────────────────────────

        private static void GenerateTerrainCollisionBoxList(
            CollisionTester tester,
            IBlockAccessor blockAccessor,
            double motionX, double motionY, double motionZ,
            float stepHeight, float yExtra, int dimension)
        {
            Cuboidd entityBox = tester.entityBox;

            bool minUnchanged = tester.minPos.SetAndEquals(
                (int)(entityBox.X1 + Math.Min(0.0, motionX)),
                (int)(entityBox.Y1 + Math.Min(0.0, motionY) - yExtra),
                (int)(entityBox.Z1 + Math.Min(0.0, motionZ)));

            double maxY = Math.Max(entityBox.Y1 + stepHeight, entityBox.Y2);
            bool maxUnchanged = tester.maxPos.SetAndEquals(
                (int)(entityBox.X2 + Math.Max(0.0, motionX)),
                (int)(maxY + Math.Max(0.0, motionY)),
                (int)(entityBox.Z2 + Math.Max(0.0, motionZ)));

            if (minUnchanged && maxUnchanged) return;

            tester.CollisionBoxList.Clear();
            tester.tmpPos.dimension = dimension;

            blockAccessor.WalkBlocks(
                tester.minPos, tester.maxPos,
                (block, x, y, z) =>
                {
                    Cuboidf[]? boxes = block.GetCollisionBoxes(blockAccessor, tester.tmpPos.Set(x, y, z));
                    if (boxes != null) tester.CollisionBoxList.Add(boxes, x, y, z, block);
                },
                centerOrder: true);
        }

        // ── dynamic collision box collection ──────────────────────────────────────

        private static List<DynamicCollisionBox> CollectDynamicCollisionBoxes(
            Entity movingEntity, int dimension,
            Cuboidd movingEntityBox,
            double motionX, double motionY, double motionZ,
            float stepHeight, float yExtra)
        {
            var results = new List<DynamicCollisionBox>();
            if (DynamicCollisionSource == null) return results;

            Cuboidd q = movingEntityBox.Clone();
            q.X1 += Math.Min(0.0, motionX);
            q.Y1 += Math.Min(0.0, motionY) - yExtra;
            q.Z1 += Math.Min(0.0, motionZ);
            q.X2 += Math.Max(0.0, motionX);
            q.Y2 = Math.Max(q.Y1 + stepHeight, q.Y2 + Math.Max(0.0, motionY));
            q.Z2 += Math.Max(0.0, motionZ);

            DynamicCollisionSource.CollectCollisionBoxes(movingEntity, q, results);
            return results;
        }

        // ── geometry utilities ────────────────────────────────────────────────────

        private static Cuboidd OffsetEntityBox(Cuboidd b, double x, double y, double z)
            => new Cuboidd(b.X1 + x, b.Y1 + y, b.Z1 + z, b.X2 + x, b.Y2 + y, b.Z2 + z);

        // SAT overlap test between an OBB and an AABB.
        private static bool IntersectsObbAabb(DynamicCollisionBox obb, Cuboidd aabb)
        {
            Vector3 aabbCenter = new Vector3(
                (float)((aabb.X1 + aabb.X2) * 0.5),
                (float)((aabb.Y1 + aabb.Y2) * 0.5),
                (float)((aabb.Z1 + aabb.Z2) * 0.5));
            Vector3 aabbHalf = new Vector3(
                (float)((aabb.X2 - aabb.X1) * 0.5),
                (float)((aabb.Y2 - aabb.Y1) * 0.5),
                (float)((aabb.Z2 - aabb.Z1) * 0.5));

            Vector3[] a =
            {
                SafeNormalize(Vector3.Transform(Vector3.UnitX, obb.Orientation), Vector3.UnitX),
                SafeNormalize(Vector3.Transform(Vector3.UnitY, obb.Orientation), Vector3.UnitY),
                SafeNormalize(Vector3.Transform(Vector3.UnitZ, obb.Orientation), Vector3.UnitZ),
            };

            float[] ea = { obb.HalfExtents.X, obb.HalfExtents.Y, obb.HalfExtents.Z };
            float[] eb = { aabbHalf.X, aabbHalf.Y, aabbHalf.Z };

            const float eps = 1e-6f;
            float[,] r = new float[3, 3];
            float[,] absR = new float[3, 3];
            for (int i = 0; i < 3; i++)
            {
                r[i, 0] = a[i].X; r[i, 1] = a[i].Y; r[i, 2] = a[i].Z;
                absR[i, 0] = MathF.Abs(r[i, 0]) + eps;
                absR[i, 1] = MathF.Abs(r[i, 1]) + eps;
                absR[i, 2] = MathF.Abs(r[i, 2]) + eps;
            }

            Vector3 tWorld = aabbCenter - obb.Center;
            float[] t = { Vector3.Dot(tWorld, a[0]), Vector3.Dot(tWorld, a[1]), Vector3.Dot(tWorld, a[2]) };

            for (int i = 0; i < 3; i++)
            {
                float ra = ea[i];
                float rb = eb[0] * absR[i, 0] + eb[1] * absR[i, 1] + eb[2] * absR[i, 2];
                if (MathF.Abs(t[i]) > ra + rb) return false;
            }
            for (int j = 0; j < 3; j++)
            {
                float ra = ea[0] * absR[0, j] + ea[1] * absR[1, j] + ea[2] * absR[2, j];
                float rb = eb[j];
                float pt = MathF.Abs(t[0] * r[0, j] + t[1] * r[1, j] + t[2] * r[2, j]);
                if (pt > ra + rb) return false;
            }
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    float ra = ea[(i + 1) % 3] * absR[(i + 2) % 3, j] + ea[(i + 2) % 3] * absR[(i + 1) % 3, j];
                    float rb = eb[(j + 1) % 3] * absR[i, (j + 2) % 3] + eb[(j + 2) % 3] * absR[i, (j + 1) % 3];
                    float pt = MathF.Abs(t[(i + 2) % 3] * r[(i + 1) % 3, j] - t[(i + 1) % 3] * r[(i + 2) % 3, j]);
                    if (pt > ra + rb) return false;
                }
            return true;
        }

        // Conservative swept AABB vs OBB using separating axis theorem.
        // Returns the earliest time of contact in [0,1] along the sweep vector.
        private static bool TrySweepAabbAgainstObb(
            Cuboidd movingAabb,
            Vector3 sweep,
            DynamicCollisionBox staticObb,
            out double hitFraction)
        {
            hitFraction = 1.0;

            Vector3 movingCenter = new Vector3(
                (float)((movingAabb.X1 + movingAabb.X2) * 0.5),
                (float)((movingAabb.Y1 + movingAabb.Y2) * 0.5),
                (float)((movingAabb.Z1 + movingAabb.Z2) * 0.5));
            Vector3 movingHalf = new Vector3(
                (float)((movingAabb.X2 - movingAabb.X1) * 0.5),
                (float)((movingAabb.Y2 - movingAabb.Y1) * 0.5),
                (float)((movingAabb.Z2 - movingAabb.Z1) * 0.5));

            Vector3[] obbAxes =
            {
                SafeNormalize(Vector3.Transform(Vector3.UnitX, staticObb.Orientation), Vector3.UnitX),
                SafeNormalize(Vector3.Transform(Vector3.UnitY, staticObb.Orientation), Vector3.UnitY),
                SafeNormalize(Vector3.Transform(Vector3.UnitZ, staticObb.Orientation), Vector3.UnitZ),
            };
            Vector3[] worldAxes = { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ };

            double enter = 0.0, exit = 1.0;

            for (int i = 0; i < 3; i++)
                if (!SweepTestAxis(obbAxes[i], movingCenter, movingHalf, sweep,
                        staticObb.Center, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                    return false;

            for (int i = 0; i < 3; i++)
                if (!SweepTestAxis(worldAxes[i], movingCenter, movingHalf, sweep,
                        staticObb.Center, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                    return false;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    Vector3 cross = Vector3.Cross(obbAxes[i], worldAxes[j]);
                    if (cross.LengthSquared() <= 1e-8f) continue;
                    cross = Vector3.Normalize(cross);
                    if (!SweepTestAxis(cross, movingCenter, movingHalf, sweep,
                            staticObb.Center, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                        return false;
                }

            if (exit < 0.0 || enter > 1.0) return false;
            hitFraction = Math.Clamp(enter, 0.0, 1.0);
            return true;
        }

        private static bool SweepTestAxis(
            Vector3 axis,
            Vector3 movingCenter, Vector3 movingHalf, Vector3 sweep,
            Vector3 staticCenter, Vector3 staticHalf, Vector3[] staticAxes,
            ref double enter, ref double exit)
        {
            const double eps = 1e-9;

            double mProj = Vector3.Dot(movingCenter, axis);
            double sProj = Vector3.Dot(staticCenter, axis);
            double vel = Vector3.Dot(sweep, axis);

            double mRad =
                Math.Abs(axis.X) * movingHalf.X +
                Math.Abs(axis.Y) * movingHalf.Y +
                Math.Abs(axis.Z) * movingHalf.Z;
            double sRad =
                Math.Abs(Vector3.Dot(axis, staticAxes[0])) * staticHalf.X +
                Math.Abs(Vector3.Dot(axis, staticAxes[1])) * staticHalf.Y +
                Math.Abs(Vector3.Dot(axis, staticAxes[2])) * staticHalf.Z;

            double mMin = mProj - mRad, mMax = mProj + mRad;
            double sMin = sProj - sRad, sMax = sProj + sRad;

            if (Math.Abs(vel) <= eps)
                return mMax >= sMin && mMin <= sMax;

            double axEnter = vel > 0.0 ? (sMin - mMax) / vel : (sMax - mMin) / vel;
            double axExit = vel > 0.0 ? (sMax - mMin) / vel : (sMin - mMax) / vel;

            if (axEnter > enter) enter = axEnter;
            if (axExit < exit) exit = axExit;
            return enter <= exit;
        }

        private static Vector3 SafeNormalize(Vector3 v, Vector3 fallback)
            => v.LengthSquared() > 1e-10f ? Vector3.Normalize(v) : fallback;
    }
}