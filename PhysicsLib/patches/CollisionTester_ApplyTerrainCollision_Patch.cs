using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using PhysicsLib.Entities.Behaviours;
using HarmonyLib;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Config;
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
        private const double SupportProbeAbove = 0.5;    // scan this far above current feet
        // large to tolerate client position lag
        // on fast-moving platforms (boats)
        private const double SupportProbeBelow = 0.12;   // scan this far below current feet

        // Surface must face at least this far upward to be a floor candidate.
        private const double MinSupportUpY = 0.65;

        // Foot-sample inset so corner samples stay inside a narrow deck plank.
        private const double SupportFootInset = 0.025;

        // Extra XZ padding when checking whether foot samples are over a collider.
        private const double SupportHorizontalPadding = 0.015;

        // Maximum distance above feetY that a snap is allowed to move the player.
        // Kept small so we don't teleport upward onto a surface that's far above.
        private const double SupportSnapUp = 0.5;    // must match SupportProbeAbove

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
            if (!SupportStates.Remove(entity.EntityId)) entity.Api.Logger.Event("Failed to removed entity: " + entity.EntityId);
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
        //The SupportEntity should be passed to this
        private static Entity? ResolveSupportEntity(Entity entity, out SupportState? supportState)
        {
            supportState = null;
            if (entity == null) return null;
            if (SupportStates.TryGetValue(entity.EntityId, out supportState) || supportState != null)
            {
                return entity.World.GetEntityById(supportState.SupportEntityId);
            }
            //entity.Api.Logger.Event("ResolveSupportEntity failed");//This currently triggers every frame
            return null;
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
            double motionX = entityPos.Motion.X * dtFactor + (carryDelta.X)*0.6;//Multiply by 0.6 to match ratio of physics-ticks(30(max))/game-ticks(50)
            double motionY = entityPos.Motion.Y * dtFactor + (carryDelta.Y)*0.6;
            double motionZ = entityPos.Motion.Z * dtFactor + (carryDelta.Z)*0.6;

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
            // pos.Y is the authoritative foot origin (set from entityPos at top of function).
            // We use pos.Y + unbiased motionY rather than entityBox.Y1 because on a moving
            // platform the client's entityPos lags the server — but that lag is already
            // compensated by carryDelta which is added to motionY above.
            double feetY = pos.Y + (motionY - biasY);

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

            // Collect dynamic boxes that overlap the probe region.
            // Pass 0 for stepHeight and yExtra so the query window is exactly the probe
            // band — passing the real stepHeight/yExtra inflates it by up to 2+ blocks
            // and causes TryGetSupportTopUnderFeet to match surfaces far above or below.
            List<DynamicCollisionBox> supportBoxes = CollectDynamicCollisionBoxes(
                entity, entityPos.Dimension, probeBox,
                0.0, 0.0, 0.0, 0f, 0f);

            // Build foot sample points at feetY — these are the same five XZ positions
            // used inside TryGetSupportTopUnderFeet, computed once here so we can test
            // each sample against every collected box and aggregate the results.
            // This is necessary for seams between two colliders: the centre sample may
            // land over box A while the corner samples land over box B.  Testing each
            // box independently means neither accumulates enough hits to return true.
            // By aggregating per-sample across all boxes we find support on seams.
            double sx1 = probeX1 + SupportFootInset;
            double sx2 = probeX2 - SupportFootInset;
            double sz1 = probeZ1 + SupportFootInset;
            double sz2 = probeZ2 - SupportFootInset;
            if (sx1 > sx2) sx1 = sx2 = (probeX1 + probeX2) * 0.5;
            if (sz1 > sz2) sz1 = sz2 = (probeZ1 + probeZ2) * 0.5;
            double scx = (probeX1 + probeX2) * 0.5;
            double scz = (probeZ1 + probeZ2) * 0.5;

            // Store foot samples as doubles — they are world-space coordinates and
            // must not be converted to float until we subtract the OBB center.
            //15 doubles per nearby vehicle
            Span<(double X, double Y, double Z)> footSamples = stackalloc (double, double, double)[5]
            {
                (scx, feetY, scz),
                (sx1, feetY, sz1),
                (sx1, feetY, sz2),
                (sx2, feetY, sz1),
                (sx2, feetY, sz2),
            };

            SupportCandidate? support = null;

            // DEBUG — log every tick while sneaking so we can diagnose seam fall-through
            bool _dbg = entity is EntityPlayer player && player.Controls?.Sneak == true && false;
            if (_dbg)
                entity.World.Logger.Debug(
                    "[SupportScan] feetY={0:F4} supportBoxes={1} samples: c=({2:F3},{3:F3}) x1z1=({4:F3},{5:F3})",
                    feetY, supportBoxes.Count, scx, scz, sx1, sz1);

            for (int i = 0; i < supportBoxes.Count; i++)
            {
                DynamicCollisionBox dyn = supportBoxes[i];
                if (!dyn.CanSupport || dyn.SourceEntity == null)
                {
                    if (_dbg)
                        entity.World.Logger.Debug("  box[{0}] SKIP canSupport={1} hasSource={2}", i, dyn.CanSupport,
                            dyn.SourceEntity != null);
                    continue;
                }

                // Reject heavily-tilted surfaces.
                Vector3 dynUp = SafeNormalize(Vector3.Transform(Vector3.UnitY, dyn.Orientation), Vector3.UnitY);
                if (dynUp.Y < MinSupportUpY)
                {
                    if (_dbg)
                        entity.World.Logger.Debug("  box[{0}] SKIP upY={1:F3} < {2:F3}", i, dynUp.Y, MinSupportUpY);
                    continue;
                }

                Quaternion invOri = Quaternion.Inverse(dyn.Orientation);
                float padX = dyn.HalfExtents.X + (float)SupportHorizontalPadding;
                float padZ = dyn.HalfExtents.Z + (float)SupportHorizontalPadding;

                if (_dbg)
                    entity.World.Logger.Debug(
                        "  box[{0}] center=({1:F3},{2:F3},{3:F3}) he=({4:F3},{5:F3},{6:F3}) padX={7:F3} padZ={8:F3}",
                        i, dyn.Center.X, dyn.Center.Y, dyn.Center.Z,
                        dyn.HalfExtents.X, dyn.HalfExtents.Y, dyn.HalfExtents.Z, padX, padZ);

                // Test each foot sample against this box independently.
                // A hit from ANY sample is enough — this is the seam fix.
                for (int s = 0; s < footSamples.Length; s++)
                {
                    // Subtract CenterD (Vec3d) in double before converting to float.
                    Vector3 relSample = new Vector3(
                        (float)(footSamples[s].X - dyn.CenterD.X),
                        (float)(footSamples[s].Y - dyn.CenterD.Y),
                        (float)(footSamples[s].Z - dyn.CenterD.Z));
                    Vector3 local = Vector3.Transform(relSample, invOri);

                    if (_dbg)
                        entity.World.Logger.Debug(
                            "    sample[{0}] local=({1:F3},{2:F3},{3:F3}) absX={4:F3}/{5:F3} absZ={6:F3}/{7:F3}",
                            s, local.X, local.Y, local.Z, MathF.Abs(local.X), padX, MathF.Abs(local.Z), padZ);

                    if (MathF.Abs(local.X) > padX) continue;
                    if (MathF.Abs(local.Z) > padZ) continue;

                    Vector3 topLocal = new Vector3(local.X, dyn.HalfExtents.Y, local.Z);
                    Vector3 topWorld = dyn.Center + Vector3.Transform(topLocal, dyn.Orientation);

                    double topY = topWorld.Y;
                    double delta = feetY - topY;

                    if (_dbg)
                        entity.World.Logger.Debug(
                            "    sample[{0}] HIT topY={1:F4} delta={2:F4} window=[{3:F4}..{4:F4}]",
                            s, topY, delta, -SupportProbeAbove, SupportProbeBelow);

                    if (delta < -SupportProbeAbove || delta > SupportProbeBelow) continue;

                    if (support == null || topY > support.TopY)
                        support = new SupportCandidate
                            { SupportEntity = dyn.SourceEntity, DynamicBox = dyn, TopY = topY };

                    break; // one hit per box is enough; move to the next box
                }
            }

            if (_dbg)
                entity.World.Logger.Debug(
                    "[SupportScan] result={0}", support != null ? $"topY={support.TopY:F4}" : "NULL — will fall");

            // Snap feet to the surface and record support.
            if (support != null)
            {
                double snapDelta = support.TopY - feetY;
                // Only snap if we are actually on or near the surface (not mid-air above it).
                if (snapDelta >= -SupportProbeBelow && snapDelta <= SupportSnapUp)
                {
                    motionY += snapDelta;
                    // Re-translate entityBox to the snapped Y so the horizontal passes
                    // and IsFloorOfSupport see the correct foot position. Without this,
                    // entityBox.Y1 stays below the surface top and IsFloorOfSupport
                    // returns false, causing hull colliders to push the player sideways.
                    entityBox.Translate(0.0, snapDelta, 0.0);
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
                // Skip any collider the entity is currently standing on — its top surface
                // is beneath the feet, so it should only block vertically, not horizontally.
                if (IsUnderFeet(dynBoxes[i], entityBox)) continue;
                // Use a tentative box offset by the motion accumulated so far in this pass.
                Cuboidd tentativeX = OffsetEntityBox(entityBox, motionX, 0.0, 0.0);
                EnumPushDirection dir = EnumPushDirection.None;
                double swept = SweepAxisObbAabb(dynBoxes[i], tentativeX, motionX, 0, ref dir);
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
                if (IsUnderFeet(dynBoxes[i], entityBox)) continue;
                Cuboidd tentativeZ = OffsetEntityBox(entityBox, 0.0, 0.0, motionZ);
                EnumPushDirection dir = EnumPushDirection.None;
                double swept = SweepAxisObbAabb(dynBoxes[i], tentativeZ, motionZ, 2, ref dir);
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

            if (resolvedSupport != null)
            {
                Cuboidd finalBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);
                Vec3d feetCenter = new Vec3d(
                    (finalBox.X1 + finalBox.X2) * 0.5,
                    finalBox.Y1,
                    (finalBox.Z1 + finalBox.Z2) * 0.5);

                DynamicPhysicsBehaviour? supportPhysics =
                    support?.SupportEntity.GetBehavior<DynamicPhysicsBehaviour>();

                if (supportPhysics != null &&
                    supportPhysics.TryTransformWorldPointToLocal(feetCenter, out Vector3 localAnchor))
                {
                    SetStandingOnEntity(entity, resolvedSupport, localAnchor);
                    entity.OnGround = true;
                }
                else
                {
                    ClearStandingOnEntity(entity);
                }
            }
            else if (prevSupport != null && prevSupportPhysics != null && prevSupportState != null)
            {
                // No support found this frame, but we had one last frame.
                // This happens when the client position lags behind a moving platform —
                // the probe misses the deck by a few cm, carryDelta goes to zero, the
                // player falls behind the boat, and they never recover.
                // If the previous support entity still exists and the player hasn't fallen
                // far enough to have genuinely left the surface, keep the support alive
                // so carryDelta continues to be applied next frame.
                if (prevSupportPhysics.TryTransformWorldPointToLocal(
                        new Vec3d(finalX, finalY, finalZ), out Vector3 retainedAnchor))
                {
                    // Check the retained anchor is still on top of the surface (not fallen through).
                    if (prevSupportPhysics.TryGetPointVelocityDelta(retainedAnchor, out _))
                    {
                        SetStandingOnEntity(entity, prevSupport, retainedAnchor);
                        //entity.Api.Logger.Event("a");
                        // Don't set OnGround — we're not confirmed on the surface this frame,
                        // just preventing carryDelta from dropping to zero.
                    }
                    else
                    {
                        ClearStandingOnEntity(entity);
                        //entity.Api.Logger.Event("b");
                    }
                }
                else
                {
                    ClearStandingOnEntity(entity);
                    //entity.Api.Logger.Event("c");
                }
            }
            else if (prevSupport != null)
            {
                ClearStandingOnEntity(entity);
                //entity.Api.Logger.Event("d");
            }
            newPosition.Set(finalX, finalY, finalZ);
        }

        // ── helper: should we skip horizontal collision against this box? ─────────
        // Returns true only when the box belongs to our support entity AND the entity's
        // feet are within the normal standing-contact band above the surface.  This
        // stops the deck's own hull colliders from pushing the player sideways.
        // Returns true if this collider is the floor directly under the entity's feet —
        // i.e. its top surface is within the normal standing-contact band below Y1.
        // Used to skip horizontal collision against surfaces the entity stands on.
        // Entity-agnostic: doesn't matter which entity owns the collider.
        private static bool IsUnderFeet(DynamicCollisionBox dynBox, Cuboidd entityBox)
        {
            if (!dynBox.CanSupport) return false;
            if (!TryGetSupportTopUnderFeet(entityBox, dynBox, out double topY)) return false;

            // delta > 0: feet above surface (standing on it)
            // delta < 0: surface above feet (player about to be snapped up onto it)
            // We skip horizontal collision in both cases — if the surface is within
            // SupportSnapUp above the feet it will be snapped onto this frame, so
            // stopping the player against its edge would prevent them walking onto it.
            double delta = entityBox.Y1 - topY;
            return delta >= -SupportSnapUp && delta <= SupportProbeBelow;
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

            Cuboidd movedBox = OffsetEntityBox(entityBox,
                axis == 0 ? motion : 0.0,
                axis == 1 ? motion : 0.0,
                axis == 2 ? motion : 0.0);

            bool startOverlap = IntersectsObbAabb(obb, entityBox);
            bool endOverlap = IntersectsObbAabb(obb, movedBox);

            // Moving away from an existing overlap — allow.
            if (startOverlap && !endOverlap) return motion;

            // Pre-existing overlap and not moving away.
            if (startOverlap && endOverlap)
            {
                if (axis == 1)
                {
                    // Vertical axis — stop.
                    direction = motion > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;
                    return 0.0;
                }

                // Horizontal axis — the entity is inside this collider already.
                // Determine whether this is a floor (contact normal mostly vertical)
                // or a wall (contact normal mostly horizontal).
                // We detect this by checking which world axis has the smallest overlap
                // via ComputePenetrationAxes. If the minimum-penetration axis is Y,
                // it's a floor and horizontal motion should pass freely.
                // If it's X or Z, it's a wall and we stop.
                if (IsContactNormalMostlyVertical(obb, entityBox))
                    return motion;  // floor — pass through horizontally

                direction = motion > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;
                return 0.0;
            }

            // No start overlap — run swept test.
            if (!endOverlap) return motion;

            Vector3 sweep = axis switch
            {
                0 => new Vector3((float)motion, 0f, 0f),
                1 => new Vector3(0f, (float)motion, 0f),
                2 => new Vector3(0f, 0f, (float)motion),
                _ => Vector3.Zero
            };

            if (!TrySweepAabbAgainstObb(entityBox, sweep, obb, out double hitFraction))
                return motion;

            double skinFraction = Math.Abs(motion) > 1e-9 ? SweepSkin / Math.Abs(motion) : 0.0;
            double safeFraction = Math.Clamp(hitFraction - skinFraction, 0.0, 1.0);

            direction = motion > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;
            return motion * safeFraction;
        }

        // Returns true if the minimum-penetration axis between the OBB and AABB is
        // predominantly vertical — meaning the entity is resting on top of this collider
        // rather than colliding with it from the side.
        private static bool IsContactNormalMostlyVertical(DynamicCollisionBox obb, Cuboidd aabb)
        {
            // Project both shapes onto each world axis and find the smallest overlap.
            // The axis with the smallest overlap is the contact normal direction.
            double minOverlap = double.MaxValue;
            int minAxis = 1; // default to Y

            Vector3 aabbHalf = new Vector3(
                (float)((aabb.X2 - aabb.X1) * 0.5),
                (float)((aabb.Y2 - aabb.Y1) * 0.5),
                (float)((aabb.Z2 - aabb.Z1) * 0.5));

            Vector3[] obbAxes =
            {
                SafeNormalize(Vector3.Transform(Vector3.UnitX, obb.Orientation), Vector3.UnitX),
                SafeNormalize(Vector3.Transform(Vector3.UnitY, obb.Orientation), Vector3.UnitY),
                SafeNormalize(Vector3.Transform(Vector3.UnitZ, obb.Orientation), Vector3.UnitZ),
            };
            Vector3[] worldAxes = { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ };

            // Test world axes only (sufficient for axis-aligned contact normal detection).
            for (int i = 0; i < 3; i++)
            {
                Vector3 axis = worldAxes[i];

                double aabbCenterProj = axis.X * ((aabb.X1 + aabb.X2) * 0.5)
                                        + axis.Y * ((aabb.Y1 + aabb.Y2) * 0.5)
                                        + axis.Z * ((aabb.Z1 + aabb.Z2) * 0.5);
                double aabbR = Math.Abs(axis.X) * aabbHalf.X + Math.Abs(axis.Y) * aabbHalf.Y + Math.Abs(axis.Z) * aabbHalf.Z;

                double obbCenterProj = axis.X * obb.CenterD.X + axis.Y * obb.CenterD.Y + axis.Z * obb.CenterD.Z;
                double obbR =
                    Math.Abs(Vector3.Dot(axis, obbAxes[0])) * obb.HalfExtents.X +
                    Math.Abs(Vector3.Dot(axis, obbAxes[1])) * obb.HalfExtents.Y +
                    Math.Abs(Vector3.Dot(axis, obbAxes[2])) * obb.HalfExtents.Z;

                double overlap = (aabbR + obbR) - Math.Abs(aabbCenterProj - obbCenterProj);
                if (overlap < minOverlap) { minOverlap = overlap; minAxis = i; }
            }

            // minAxis == 1 means Y is the minimum-penetration axis — contact normal is vertical.
            return minAxis == 1;
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

            // Store samples as double tuples — world-space coords, must not be cast
            // to float until after subtracting the OBB center.
            Span<(double X, double Y, double Z)> samples = stackalloc (double, double, double)[5]
            {
                (cx, fy, cz),
                (x1, fy, z1),
                (x1, fy, z2),
                (x2, fy, z1),
                (x2, fy, z2),
            };

            bool found = false;
            for (int i = 0; i < samples.Length; i++)
            {
                // Subtract CenterD (Vec3d) in double, then convert to float for the rotation.
                Vector3 relSample = new Vector3(
                    (float)(samples[i].X - dynBox.CenterD.X),
                    (float)(samples[i].Y - dynBox.CenterD.Y),
                    (float)(samples[i].Z - dynBox.CenterD.Z));
                Vector3 local = Vector3.Transform(relSample, invOrientation);

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
            q.Z1 += Math.Min(0.0, motionZ);
            q.X2 += Math.Max(0.0, motionX);
            q.Z2 += Math.Max(0.0, motionZ);
            // Y2 must be computed before Y1 is modified, otherwise q.Y1 + stepHeight
            // uses the already-lowered Y1 and can produce a smaller range than intended.
            q.Y2 = Math.Max(movingEntityBox.Y2 + Math.Max(0.0, motionY),
                movingEntityBox.Y1 + stepHeight);
            q.Y1 += Math.Min(0.0, motionY) - yExtra;

            DynamicCollisionSource.CollectCollisionBoxes(movingEntity, q, ref results);
            return results;
        }

        // ── geometry utilities ────────────────────────────────────────────────────

        private static Cuboidd OffsetEntityBox(Cuboidd b, double x, double y, double z)
            => new Cuboidd(b.X1 + x, b.Y1 + y, b.Z1 + z, b.X2 + x, b.Y2 + y, b.Z2 + z);

        // SAT overlap test between an OBB and an AABB.
        // All separating-axis arithmetic is done in double precision.
        // The OBB center and AABB center are large world-space coordinates (~500000),
        // so computing their difference in float loses ~0.03 units of precision —
        // enough to make edge colliders appear non-solid.
        private static bool IntersectsObbAabb(DynamicCollisionBox obb, Cuboidd aabb)
        {
            // AABB extents — differences of same-magnitude numbers, safe in double.
            double aabbHalfX = (aabb.X2 - aabb.X1) * 0.5;
            double aabbHalfY = (aabb.Y2 - aabb.Y1) * 0.5;
            double aabbHalfZ = (aabb.Z2 - aabb.Z1) * 0.5;
            double aabbCx = (aabb.X1 + aabb.X2) * 0.5;
            double aabbCy = (aabb.Y1 + aabb.Y2) * 0.5;
            double aabbCz = (aabb.Z1 + aabb.Z2) * 0.5;

            // OBB axes — unit vectors, fine as float.
            Vector3 ax = SafeNormalize(Vector3.Transform(Vector3.UnitX, obb.Orientation), Vector3.UnitX);
            Vector3 ay = SafeNormalize(Vector3.Transform(Vector3.UnitY, obb.Orientation), Vector3.UnitY);
            Vector3 az = SafeNormalize(Vector3.Transform(Vector3.UnitZ, obb.Orientation), Vector3.UnitZ);

            double ea0 = obb.HalfExtents.X, ea1 = obb.HalfExtents.Y, ea2 = obb.HalfExtents.Z;
            double eb0 = aabbHalfX, eb1 = aabbHalfY, eb2 = aabbHalfZ;

            // Translation vector in double — subtract using CenterD (Vec3d) not Center (Vector3/float)
            // to avoid precision loss at large world coordinates (~513000).
            double tx = aabbCx - obb.CenterD.X;
            double ty = aabbCy - obb.CenterD.Y;
            double tz = aabbCz - obb.CenterD.Z;

            // Project translation onto OBB axes.
            double t0 = tx * ax.X + ty * ax.Y + tz * ax.Z;
            double t1 = tx * ay.X + ty * ay.Y + tz * ay.Z;
            double t2 = tx * az.X + ty * az.Y + tz * az.Z;

            const double eps = 1e-6;

            // Rotation matrix entries and abs versions.
            double r00 = ax.X, r01 = ax.Y, r02 = ax.Z;
            double r10 = ay.X, r11 = ay.Y, r12 = ay.Z;
            double r20 = az.X, r21 = az.Y, r22 = az.Z;

            double ar00 = Math.Abs(r00) + eps, ar01 = Math.Abs(r01) + eps, ar02 = Math.Abs(r02) + eps;
            double ar10 = Math.Abs(r10) + eps, ar11 = Math.Abs(r11) + eps, ar12 = Math.Abs(r12) + eps;
            double ar20 = Math.Abs(r20) + eps, ar21 = Math.Abs(r21) + eps, ar22 = Math.Abs(r22) + eps;

            // OBB face axes.
            if (Math.Abs(t0) > ea0 + eb0 * ar00 + eb1 * ar01 + eb2 * ar02) return false;
            if (Math.Abs(t1) > ea1 + eb0 * ar10 + eb1 * ar11 + eb2 * ar12) return false;
            if (Math.Abs(t2) > ea2 + eb0 * ar20 + eb1 * ar21 + eb2 * ar22) return false;

            // AABB face axes.
            if (Math.Abs(t0 * r00 + t1 * r10 + t2 * r20) > ea0 * ar00 + ea1 * ar10 + ea2 * ar20 + eb0) return false;
            if (Math.Abs(t0 * r01 + t1 * r11 + t2 * r21) > ea0 * ar01 + ea1 * ar11 + ea2 * ar21 + eb1) return false;
            if (Math.Abs(t0 * r02 + t1 * r12 + t2 * r22) > ea0 * ar02 + ea1 * ar12 + ea2 * ar22 + eb2) return false;

            // Cross-product axes.
            if (Math.Abs(t2 * r10 - t1 * r20) > ea1 * ar20 + ea2 * ar10 + eb1 * ar02 + eb2 * ar01) return false;
            if (Math.Abs(t2 * r11 - t1 * r21) > ea1 * ar21 + ea2 * ar11 + eb0 * ar02 + eb2 * ar00) return false;
            if (Math.Abs(t2 * r12 - t1 * r22) > ea1 * ar22 + ea2 * ar12 + eb0 * ar01 + eb1 * ar00) return false;
            if (Math.Abs(t0 * r20 - t2 * r00) > ea0 * ar20 + ea2 * ar00 + eb1 * ar12 + eb2 * ar11) return false;
            if (Math.Abs(t0 * r21 - t2 * r01) > ea0 * ar21 + ea2 * ar01 + eb0 * ar12 + eb2 * ar10) return false;
            if (Math.Abs(t0 * r22 - t2 * r02) > ea0 * ar22 + ea2 * ar02 + eb0 * ar11 + eb1 * ar10) return false;
            if (Math.Abs(t1 * r00 - t0 * r10) > ea0 * ar10 + ea1 * ar00 + eb1 * ar22 + eb2 * ar21) return false;
            if (Math.Abs(t1 * r01 - t0 * r11) > ea0 * ar11 + ea1 * ar01 + eb0 * ar22 + eb2 * ar20) return false;
            if (Math.Abs(t1 * r02 - t0 * r12) > ea0 * ar12 + ea1 * ar02 + eb0 * ar21 + eb1 * ar20) return false;

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

            // Compute center relative to OBB center in double using CenterD (Vec3d).
            Vector3 movingCenter = new Vector3(
                (float)((movingAabb.X1 + movingAabb.X2) * 0.5 - staticObb.CenterD.X),
                (float)((movingAabb.Y1 + movingAabb.Y2) * 0.5 - staticObb.CenterD.Y),
                (float)((movingAabb.Z1 + movingAabb.Z2) * 0.5 - staticObb.CenterD.Z));
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

            // staticCenter is Vector3.Zero because movingCenter is already expressed
            // relative to the OBB center.
            for (int i = 0; i < 3; i++)
                if (!SweepTestAxis(obbAxes[i], movingCenter, movingHalf, sweep,
                        Vector3.Zero, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                    return false;

            for (int i = 0; i < 3; i++)
                if (!SweepTestAxis(worldAxes[i], movingCenter, movingHalf, sweep,
                        Vector3.Zero, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                    return false;

            for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                Vector3 cross = Vector3.Cross(obbAxes[i], worldAxes[j]);
                if (cross.LengthSquared() <= 1e-8f) continue;
                cross = Vector3.Normalize(cross);
                if (!SweepTestAxis(cross, movingCenter, movingHalf, sweep,
                        Vector3.Zero, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
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