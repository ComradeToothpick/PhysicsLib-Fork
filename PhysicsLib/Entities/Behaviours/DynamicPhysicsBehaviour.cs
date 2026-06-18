using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using PhysicsLib.Client;
using PhysicsLib.patches;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace PhysicsLib.Entities.Behaviours
{
    public class DynamicPhysicsBehaviour : PhysicsBehaviorBase
    {
        private ICoreAPI? api;
        private PhysicsLibModSystem? physics;
        private string[]? selectors;
        private DebugRenderer? debugRenderer;

        private Vector3 localCenterOfMassOffset;
        private readonly List<ManualChildBox> manualChildBoxes = new();
        private readonly List<DynamicCollisionBox> cachedWorldBoxes = new();

        private Vector3 previousBodyPosition;
        private Quaternion previousBodyOrientation = Quaternion.Identity;
        private Vector3 currentBodyPosition;
        private Quaternion currentBodyOrientation = Quaternion.Identity;
        private bool hasPreviousPose;

        private Vector3 cachedPosePosition;
        private Quaternion cachedPoseOrientation = Quaternion.Identity;
        private bool cacheValid;

        // Double-precision body position — kept alongside the float version so that
        // world-space center computation doesn't lose precision at large coordinates.
        // bodyPosition (float) is only used for rotation/offset math where values are small.
        private Vec3d bodyPositionD = new Vec3d();
        private Vec3d previousBodyPositionD = new Vec3d();
        private Vec3d currentBodyPositionD = new Vec3d();

        public Vec3f velocity = new Vec3f();

        public DynamicPhysicsBehaviour(Entity entity) : base(entity)
        {
        }

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (entity.Api.Side == EnumAppSide.Client)
            {
                capi = entity.Api as ICoreClientAPI;
                debugRenderer = new DebugRenderer(this);
                capi.Event.RegisterRenderer(debugRenderer, EnumRenderStage.AfterFinalComposition);
            }

            selectors = attributes["selectors"].AsArray<string>();

            api = entity.Api;
            physics = api.ModLoader.GetModSystem<PhysicsLibModSystem>();

            (CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource as DynamicCollisionSource)?.Register(this);

            var shape = entity.Properties.Client.Shape;
            var shapeLoc = shape.Base.Clone();
            shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";

            BuiltCompound? cachedShape = physics.TryGetCompoundShape(shapeLoc.Path);

            if (cachedShape == null)
            {
                var asset = api.Assets.TryGet(shapeLoc);
                if (asset == null)
                {
                    api.Logger.Warning("[physicslib] Missing shape asset {0} for entity {1}", shapeLoc, entity.Code);
                    return;
                }

                var compoundShape = asset.ToObject<Shape>();
                if (compoundShape == null || compoundShape.Elements == null || compoundShape.Elements.Length == 0)
                {
                    api.Logger.Warning("[physicslib] Entity {0} has no loaded shape elements.", entity.Code);
                    return;
                }

                var built = BuildCompoundFromShape(compoundShape);
                cachedShape = physics.AddCompoundShape(shapeLoc.Path, built);
            }

            localCenterOfMassOffset = cachedShape.Value.LocalCenterOfMassOffset;

            manualChildBoxes.Clear();
            manualChildBoxes.AddRange(cachedShape.Value.ManualChildBoxes);

            cacheValid = false;
        }

        public override void OnEntityDespawn(EntityDespawnData despawn)
        {
            base.OnEntityDespawn(despawn);

            (CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource as DynamicCollisionSource)?.Unregister(this);
            if (debugRenderer != null && capi != null)
            {
                capi.Event.UnregisterRenderer(debugRenderer, EnumRenderStage.AfterFinalComposition);
                debugRenderer = null;
            }
        }

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            if (api == null || !entity.Alive)
                return;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return;

            if (!hasPreviousPose)
            {
                previousBodyPosition = bodyPosition;
                previousBodyOrientation = bodyOrientation;
                currentBodyPosition = bodyPosition;
                currentBodyOrientation = bodyOrientation;
                previousBodyPositionD.Set(bodyPosD);
                currentBodyPositionD.Set(bodyPosD);
                hasPreviousPose = true;
                velocity.Set(0, 0, 0);
            }
            else
            {
                previousBodyPosition = currentBodyPosition;
                previousBodyOrientation = currentBodyOrientation;
                previousBodyPositionD.Set(currentBodyPositionD);

                currentBodyPosition = bodyPosition;
                currentBodyOrientation = bodyOrientation;
                currentBodyPositionD.Set(bodyPosD);

                Vector3 frameDelta = currentBodyPosition - previousBodyPosition;

                if (deltaTime > 1e-6f)
                {
                    velocity.Set(
                        frameDelta.X / deltaTime,
                        frameDelta.Y / deltaTime,
                        frameDelta.Z / deltaTime
                    );
                }
                else
                {
                    velocity.Set(0, 0, 0);
                }
            }

            bodyPositionD.Set(bodyPosD);
            RebuildWorldCollisionCacheIfNeeded(bodyPosition, bodyPosD, bodyOrientation);
        }

        public override string PropertyName() => "bepu-physics";

        public void AppendDynamicCollisionBoxes(Cuboidd queryBox, List<DynamicCollisionBox> results)
        {
            if (manualChildBoxes.Count == 0)
                return;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return;

            RebuildWorldCollisionCacheIfNeeded(bodyPosition, bodyPosD, bodyOrientation);

            for (int i = 0; i < cachedWorldBoxes.Count; i++)
            {
                if (cachedWorldBoxes[i].Box.IntersectsOrTouches(queryBox))
                {
                    results.Add(cachedWorldBoxes[i]);
                }
            }
        }

        public bool TryTransformWorldPointToLocal(Vec3d worldPoint, out Vector3 localPoint)
        {
            localPoint = Vector3.Zero;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return false;

            // Subtract in double to preserve precision at large world coordinates.
            Vector3 p = new Vector3(
                (float)(worldPoint.X - bodyPosD.X),
                (float)(worldPoint.Y - bodyPosD.Y),
                (float)(worldPoint.Z - bodyPosD.Z));

            Quaternion inv = Quaternion.Inverse(bodyOrientation);
            localPoint = Vector3.Transform(p, inv);
            return true;
        }

        public bool TryGetPointVelocityDelta(Vector3 localPoint, out Vec3d delta)
        {
            delta = new Vec3d();

            if (!hasPreviousPose)
                return false;

            Quaternion previousYaw = ExtractYawOnly(previousBodyOrientation);
            Quaternion currentYaw = ExtractYawOnly(currentBodyOrientation);

            Vector3 previousWorldPoint =
                previousBodyPosition + Vector3.Transform(localPoint, previousYaw);

            Vector3 currentWorldPoint =
                currentBodyPosition + Vector3.Transform(localPoint, currentYaw);

            Vector3 d = currentWorldPoint - previousWorldPoint;

            delta.Set(d.X, d.Y, d.Z);
            return true;
        }

        public bool TryGetSupportTopYUnderBox(Cuboidd entityBox, double horizontalPadding, out double supportTopY)
        {
            supportTopY = 0.0;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return false;

            RebuildWorldCollisionCacheIfNeeded(bodyPosition, bodyPosD, bodyOrientation);

            bool found = false;

            for (int i = 0; i < cachedWorldBoxes.Count; i++)
            {
                Cuboidd b = cachedWorldBoxes[i].Box;

                bool overlapsHorizontally =
                    entityBox.X2 > b.X1 - horizontalPadding &&
                    entityBox.X1 < b.X2 + horizontalPadding &&
                    entityBox.Z2 > b.Z1 - horizontalPadding &&
                    entityBox.Z1 < b.Z2 + horizontalPadding;

                if (!overlapsHorizontally)
                    continue;

                if (!found || b.Y2 > supportTopY)
                {
                    supportTopY = b.Y2;
                    found = true;
                }
            }

            return found;
        }

        private void RebuildWorldCollisionCacheIfNeeded(Vector3 bodyPosition, Vec3d bodyPosD, Quaternion bodyOrientation)
        {
            if (cacheValid &&
                Vector3.DistanceSquared(bodyPosition, cachedPosePosition) < 1e-10f &&
                MathF.Abs(Quaternion.Dot(bodyOrientation, cachedPoseOrientation)) > 0.999999f)
            {
                return;
            }

            cachedPosePosition = bodyPosition;
            cachedPoseOrientation = bodyOrientation;
            cacheValid = true;

            cachedWorldBoxes.Clear();

            Matrix4x4 bodyRotationMatrix = Matrix4x4.CreateFromQuaternion(bodyOrientation);

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                ManualChildBox child = manualChildBoxes[i];

                // Compute world center in double to avoid float precision loss at large
                // world coordinates (~513000). bodyPosD is Vec3d; child.LocalPosition is
                // a small local offset (safe as float).
                Vector3 localOffset = Vector3.Transform(child.LocalPosition, bodyOrientation);
                Vec3d childWorldCenterD = new Vec3d(
                    bodyPosD.X + localOffset.X,
                    bodyPosD.Y + localOffset.Y,
                    bodyPosD.Z + localOffset.Z);

                // Float center only used for rotation math — values are small (local offsets).
                Vector3 childWorldCenter = new Vector3(
                    (float)childWorldCenterD.X,
                    (float)childWorldCenterD.Y,
                    (float)childWorldCenterD.Z);

                Matrix4x4 childLocalRotationMatrix = Matrix4x4.CreateFromQuaternion(child.LocalOrientation);
                Quaternion childWorldOrientation = ExtractPureRotation(childLocalRotationMatrix * bodyRotationMatrix);

                Cuboidd broadphaseAabb = CreateAabbFromOrientedBox(
                    childWorldCenterD,
                    childWorldOrientation,
                    child.HalfExtents
                );

                cachedWorldBoxes.Add(new DynamicCollisionBox
                {
                    Box = broadphaseAabb,
                    Center = childWorldCenter,
                    CenterD = childWorldCenterD,
                    Orientation = childWorldOrientation,
                    HalfExtents = child.HalfExtents,
                    SourceEntity = entity,
                    CanSupport = true
                });
            }
        }

        private static Quaternion ExtractYawOnly(Quaternion q)
        {
            Vector3 forward = Vector3.Transform(Vector3.UnitZ, q);
            forward.Y = 0f;

            if (forward.LengthSquared() < 1e-10f)
                return Quaternion.Identity;

            forward = Vector3.Normalize(forward);
            float yaw = MathF.Atan2(forward.X, forward.Z);
            return Quaternion.CreateFromAxisAngle(Vector3.UnitY, yaw);
        }

        private static Vector3 ToVector3(double x, double y, double z)
        {
            return new Vector3((float)x, (float)y, (float)z);
        }

        private static Vector3 TransformPoint(Matrix4x4 m, Vector3 p)
        {
            return Vector3.Transform(p, m);
        }

        private static Vector3 TransformDirection(Matrix4x4 m, Vector3 v)
        {
            return Vector3.TransformNormal(v, m);
        }

        private static Quaternion ExtractPureRotation(Matrix4x4 m)
        {
            Vector3 x = TransformDirection(m, Vector3.UnitX);
            Vector3 y = TransformDirection(m, Vector3.UnitY);
            Vector3 z = TransformDirection(m, Vector3.UnitZ);

            if (x.LengthSquared() <= 1e-10f || y.LengthSquared() <= 1e-10f || z.LengthSquared() <= 1e-10f)
                return Quaternion.Identity;

            x = Vector3.Normalize(x);
            y = Vector3.Normalize(y);
            z = Vector3.Normalize(Vector3.Cross(x, y));

            if (z.LengthSquared() <= 1e-10f)
                return Quaternion.Identity;

            y = Vector3.Normalize(Vector3.Cross(z, x));

            Matrix4x4 rot = new Matrix4x4(
                x.X, x.Y, x.Z, 0f,
                y.X, y.Y, y.Z, 0f,
                z.X, z.Y, z.Z, 0f,
                0f, 0f, 0f, 1f
            );

            return Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(rot));
        }

        private static Vector3 ExtractAxisScale(Matrix4x4 m)
        {
            Vector3 x = TransformDirection(m, Vector3.UnitX);
            Vector3 y = TransformDirection(m, Vector3.UnitY);
            Vector3 z = TransformDirection(m, Vector3.UnitZ);

            return new Vector3(x.Length(), y.Length(), z.Length());
        }

        private static float DegreesToRadians(float degrees) => degrees * (MathF.PI / 180f);

        private static bool ElementHasRealBox(ShapeElement elem)
        {
            if (elem.From == null || elem.To == null)
                return false;

            return MathF.Abs((float)elem.To[0] - (float)elem.From[0]) > 1e-5f &&
                   MathF.Abs((float)elem.To[1] - (float)elem.From[1]) > 1e-5f &&
                   MathF.Abs((float)elem.To[2] - (float)elem.From[2]) > 1e-5f;
        }

        private static Matrix4x4 CreateVsElementLocalMatrix(ShapeElement elem, float parentXSign)
        {
            float ox = elem.RotationOrigin != null ? (float)elem.RotationOrigin[0] / 16f : 0f;
            float oy = elem.RotationOrigin != null ? (float)elem.RotationOrigin[1] / 16f : 0f;
            float oz = elem.RotationOrigin != null ? (float)elem.RotationOrigin[2] / 16f : 0f;

            float fx, fy, fz;
            if (ElementHasRealBox(elem))
            {
                fx = (float)elem.From![0] / 16f;
                fy = (float)elem.From![1] / 16f;
                fz = (float)elem.From![2] / 16f;
            }
            else
            {
                fx = ox; fy = oy; fz = oz;
            }

            float sx = elem.ScaleX == 0.0 ? 1f : (float)elem.ScaleX;
            float sy = elem.ScaleY == 0.0 ? 1f : (float)elem.ScaleY;
            float sz = elem.ScaleZ == 0.0 ? 1f : (float)elem.ScaleZ;

            Matrix4x4 translateFromMinusOrigin = Matrix4x4.CreateTranslation(fx - ox, fy - oy, fz - oz);
            Matrix4x4 scale = Matrix4x4.CreateScale(sx, sy, sz);
            Matrix4x4 rotateX = elem.RotationX != 0.0 ? Matrix4x4.CreateRotationX(DegreesToRadians((float)elem.RotationX)) : Matrix4x4.Identity;
            Matrix4x4 rotateY = elem.RotationY != 0.0 ? Matrix4x4.CreateRotationY(DegreesToRadians((float)elem.RotationY)) : Matrix4x4.Identity;
            Matrix4x4 rotateZ = elem.RotationZ != 0.0 ? Matrix4x4.CreateRotationZ(DegreesToRadians((float)elem.RotationZ)) : Matrix4x4.Identity;
            Matrix4x4 translateOrigin = Matrix4x4.CreateTranslation(ox, oy, oz);

            Matrix4x4 rotations = rotateZ * rotateY * rotateX;

            return translateFromMinusOrigin * scale * rotations * translateOrigin;
        }

        private BuiltCompound BuildCompoundFromShape(Shape shape)
        {
            var childMasses = new List<float>();
            var manualBoxes = new List<ManualChildBox>();

            if (shape.Elements == null || shape.Elements.Length == 0)
                throw new InvalidOperationException("Shape has no root elements.");

            for (int i = 0; i < shape.Elements.Length; i++)
            {
                ShapeElement root = shape.Elements[i];
                string rootPath = root.Name ?? string.Empty;

                AppendSelectedElementsRecursive(
                    root,
                    Matrix4x4.Identity,
                    rootPath,
                    false,
                    childMasses,
                    manualBoxes
                );
            }

            if (manualBoxes.Count == 0)
                throw new InvalidOperationException("Shape produced no collider children.");

            Vector3 centerOfMass = ComputeCenterOfMass(manualBoxes, childMasses);

            for (int i = 0; i < manualBoxes.Count; i++)
            {
                ManualChildBox box = manualBoxes[i];
                box.LocalPosition -= centerOfMass;
                manualBoxes[i] = box;
            }

            return new BuiltCompound
            {
                LocalCenterOfMassOffset = centerOfMass,
                ManualChildBoxes = manualBoxes,
            };
        }

        private void AppendSelectedElementsRecursive(
            ShapeElement elem,
            Matrix4x4 parentWorld,
            string path,
            bool parentSelected,
            List<float> childMasses,
            List<ManualChildBox> manualBoxes)
        {
            bool selected = parentSelected || MatchesAnySelector(path);

            float parentXSign = parentWorld.M11;

            Matrix4x4 local = CreateVsElementLocalMatrix(elem, parentXSign);
            Matrix4x4 world = local * parentWorld;

            if (selected && ElementHasRealBox(elem))
            {
                float fx = (float)elem.From![0] / 16f;
                float fy = (float)elem.From![1] / 16f;
                float fz = (float)elem.From![2] / 16f;

                float tx = (float)elem.To![0] / 16f;
                float ty = (float)elem.To![1] / 16f;
                float tz = (float)elem.To![2] / 16f;

                float width = MathF.Abs(tx - fx);
                float height = MathF.Abs(ty - fy);
                float length = MathF.Abs(tz - fz);

                if (width > 1e-5f && height > 1e-5f && length > 1e-5f)
                {
                    Vector3 elementLocalCenter = new Vector3(
                        width * 0.5f,
                        height * 0.5f,
                        length * 0.5f
                    );

                    Vector3 childPosition = TransformPoint(world, elementLocalCenter);
                    Quaternion childOrientation = ExtractPureRotation(world);
                    Vector3 axisScale = ExtractAxisScale(world);

                    Vector3 halfExtents = new Vector3(
                        width * 0.5f * axisScale.X,
                        height * 0.5f * axisScale.Y,
                        length * 0.5f * axisScale.Z
                    );

                    manualBoxes.Add(new ManualChildBox
                    {
                        LocalPosition = childPosition,
                        LocalOrientation = childOrientation,
                        HalfExtents = halfExtents
                    });

                    childMasses.Add(halfExtents.X * 2f * halfExtents.Y * 2f * halfExtents.Z * 2f);
                }
            }

            if (elem.Children == null || elem.Children.Length == 0)
                return;

            for (int i = 0; i < elem.Children.Length; i++)
            {
                ShapeElement child = elem.Children[i];
                string childName = child.Name ?? string.Empty;
                string childPath = path.Length == 0 ? childName : path + "/" + childName;

                AppendSelectedElementsRecursive(
                    child,
                    world,
                    childPath,
                    selected,
                    childMasses,
                    manualBoxes
                );
            }
        }

        private bool MatchesAnySelector(string path)
        {
            if (selectors == null || selectors.Length == 0)
                return true;

            for (int i = 0; i < selectors.Length; i++)
            {
                string selector = selectors[i];
                if (string.IsNullOrWhiteSpace(selector))
                    continue;

                selector = selector.Trim();

                if (selector.EndsWith("/*", StringComparison.Ordinal))
                {
                    string prefix = selector.Substring(0, selector.Length - 2);
                    if (path.StartsWith(prefix + "/", StringComparison.Ordinal))
                        return true;
                }

                if (selector.EndsWith("/**", StringComparison.Ordinal))
                {
                    string prefix = selector.Substring(0, selector.Length - 3);
                    if (path == prefix || path.StartsWith(prefix + "/", StringComparison.Ordinal))
                        return true;
                }

                if (WildcardMatch(path, selector))
                    return true;
            }

            return false;
        }

        private static bool WildcardMatch(string text, string pattern)
        {
            int t = 0, p = 0, star = -1, match = 0;

            while (t < text.Length)
            {
                if (p < pattern.Length && (pattern[p] == '?' || pattern[p] == text[t])) { t++; p++; continue; }
                if (p < pattern.Length && pattern[p] == '*') { star = p++; match = t; continue; }
                if (star != -1) { p = star + 1; t = ++match; continue; }
                return false;
            }

            while (p < pattern.Length && pattern[p] == '*') p++;
            return p == pattern.Length;
        }

        private Vector3 ComputeCenterOfMass(List<ManualChildBox> children, List<float> childMasses)
        {
            float totalMass = 0f;
            Vector3 weightedSum = Vector3.Zero;

            for (int i = 0; i < children.Count; i++)
            {
                float mass = childMasses[i];
                totalMass += mass;
                weightedSum += children[i].LocalPosition * mass;
            }

            if (totalMass <= 0f)
                throw new InvalidOperationException("Compound total mass must be > 0.");

            return weightedSum / totalMass;
        }

        private bool TryGetCollisionPose(out Vector3 bodyPosition, out Vec3d bodyPositionDouble, out Quaternion bodyOrientation)
        {
            bodyPosition = Vector3.Zero;
            bodyPositionDouble = new Vec3d();
            bodyOrientation = Quaternion.Identity;

            if (entity?.Pos == null)
                return false;

            EntityPos pos = entity.Pos;

            Matrix4x4 correctionMatrix = Matrix4x4.CreateRotationY(MathF.PI / 2f);
            Matrix4x4 entityRotationMatrix = Matrix4x4.CreateFromYawPitchRoll(
                pos.Yaw,
                pos.Pitch,
                pos.Roll
            );

            bodyOrientation = ExtractPureRotation(correctionMatrix * entityRotationMatrix);

            // Keep the anchor correction as a small local float offset, then add to the
            // double-precision entity origin. This avoids casting pos.X/Y/Z to float.
            Vector3 localAnchorCorrection = new Vector3(-0.5f, 0f, -0.5f);
            Vector3 localOffset = Vector3.Transform(localAnchorCorrection + localCenterOfMassOffset, bodyOrientation);

            bodyPositionDouble.Set(
                pos.X + localOffset.X,
                pos.Y + localOffset.Y,
                pos.Z + localOffset.Z);

            // Float version for rotation math only — values are relative/small elsewhere.
            bodyPosition = new Vector3(
                (float)bodyPositionDouble.X,
                (float)bodyPositionDouble.Y,
                (float)bodyPositionDouble.Z);

            return true;
        }

        private static Cuboidd CreateAabbFromOrientedBox(
            Vec3d center,
            Quaternion orientation,
            Vector3 halfExtents)
        {
            Vector3 right = Vector3.Transform(Vector3.UnitX, orientation);
            Vector3 up = Vector3.Transform(Vector3.UnitY, orientation);
            Vector3 forward = Vector3.Transform(Vector3.UnitZ, orientation);

            Vector3 extents = new Vector3(
                MathF.Abs(right.X) * halfExtents.X + MathF.Abs(up.X) * halfExtents.Y + MathF.Abs(forward.X) * halfExtents.Z,
                MathF.Abs(right.Y) * halfExtents.X + MathF.Abs(up.Y) * halfExtents.Y + MathF.Abs(forward.Y) * halfExtents.Z,
                MathF.Abs(right.Z) * halfExtents.X + MathF.Abs(up.Z) * halfExtents.Y + MathF.Abs(forward.Z) * halfExtents.Z
            );

            return new Cuboidd(
                center.X - extents.X, center.Y - extents.Y, center.Z - extents.Z,
                center.X + extents.X, center.Y + extents.Y, center.Z + extents.Z
            );
        }

        public void DebugRender(ICoreClientAPI capi)
        {
            if (capi == null || entity == null || !entity.Alive)
                return;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return;

            RebuildWorldCollisionCacheIfNeeded(bodyPosition, bodyPosD, bodyOrientation);

            int color = ColorUtil.ToRgba(255, 0, 255, 0);

            for (int i = 0; i < cachedWorldBoxes.Count; i++)
            {
                DynamicCollisionBox box = cachedWorldBoxes[i];
                DrawOrientedBoxOutline(capi, box.Center, box.Orientation, box.HalfExtents, color);
            }
        }

        private static void DrawOrientedBoxOutline(
            ICoreClientAPI capi,
            Vector3 center,
            Quaternion orientation,
            Vector3 halfExtents,
            int color)
        {
            Vector3[] corners =
            {
                TransformObbCorner(center, orientation, halfExtents, -1f, -1f, -1f),
                TransformObbCorner(center, orientation, halfExtents,  1f, -1f, -1f),
                TransformObbCorner(center, orientation, halfExtents,  1f, -1f,  1f),
                TransformObbCorner(center, orientation, halfExtents, -1f, -1f,  1f),
                TransformObbCorner(center, orientation, halfExtents, -1f,  1f, -1f),
                TransformObbCorner(center, orientation, halfExtents,  1f,  1f, -1f),
                TransformObbCorner(center, orientation, halfExtents,  1f,  1f,  1f),
                TransformObbCorner(center, orientation, halfExtents, -1f,  1f,  1f)
            };

            DrawWorldLine(capi, corners[0], corners[1], color);
            DrawWorldLine(capi, corners[1], corners[2], color);
            DrawWorldLine(capi, corners[2], corners[3], color);
            DrawWorldLine(capi, corners[3], corners[0], color);
            DrawWorldLine(capi, corners[4], corners[5], color);
            DrawWorldLine(capi, corners[5], corners[6], color);
            DrawWorldLine(capi, corners[6], corners[7], color);
            DrawWorldLine(capi, corners[7], corners[4], color);
            DrawWorldLine(capi, corners[0], corners[4], color);
            DrawWorldLine(capi, corners[1], corners[5], color);
            DrawWorldLine(capi, corners[2], corners[6], color);
            DrawWorldLine(capi, corners[3], corners[7], color);
        }

        private static Vector3 TransformObbCorner(
            Vector3 center, Quaternion orientation, Vector3 halfExtents,
            float sx, float sy, float sz)
        {
            return center + Vector3.Transform(
                new Vector3(halfExtents.X * sx, halfExtents.Y * sy, halfExtents.Z * sz),
                orientation);
        }

        private static void DrawWorldLine(ICoreClientAPI capi, Vector3 a, Vector3 b, int color)
        {
            BlockPos origin = new BlockPos(
                (int)Math.Floor(a.X),
                (int)Math.Floor(a.Y),
                (int)Math.Floor(a.Z));

            capi.Render.RenderLine(
                origin,
                (float)(a.X - origin.X), (float)(a.Y - origin.Y), (float)(a.Z - origin.Z),
                (float)(b.X - origin.X), (float)(b.Y - origin.Y), (float)(b.Z - origin.Z),
                color);
        }
    }
}