using PhysicsLib.Api;
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
                // capi.Event.RegisterRenderer(new DebugRenderer(this), EnumRenderStage.AfterFinalComposition);
            }

            selectors = attributes["selectors"].AsArray<string>();

            api = entity.Api;
            physics = api.ModLoader.GetModSystem<PhysicsLibModSystem>();

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

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            if (api == null || !entity.Alive)
                return;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Quaternion bodyOrientation))
                return;

            if (!hasPreviousPose)
            {
                previousBodyPosition = bodyPosition;
                previousBodyOrientation = bodyOrientation;
                currentBodyPosition = bodyPosition;
                currentBodyOrientation = bodyOrientation;
                hasPreviousPose = true;
                velocity.Set(0, 0, 0);
            }
            else
            {
                previousBodyPosition = currentBodyPosition;
                previousBodyOrientation = currentBodyOrientation;

                currentBodyPosition = bodyPosition;
                currentBodyOrientation = bodyOrientation;

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

            RebuildWorldCollisionCacheIfNeeded(bodyPosition, bodyOrientation);
        }

        public override string PropertyName() => "bepu-physics";

        public void AppendDynamicCollisionBoxes(Cuboidd queryBox, List<DynamicCollisionBox> results)
        {
            if (manualChildBoxes.Count == 0)
                return;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Quaternion bodyOrientation))
                return;

            RebuildWorldCollisionCacheIfNeeded(bodyPosition, bodyOrientation);

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

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Quaternion bodyOrientation))
                return false;

            Vector3 p = new Vector3((float)worldPoint.X, (float)worldPoint.Y, (float)worldPoint.Z);
            Quaternion inv = Quaternion.Inverse(bodyOrientation);

            localPoint = Vector3.Transform(p - bodyPosition, inv);
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

        public void DebugRender(ICoreClientAPI capi)
        {
            if (manualChildBoxes.Count == 0) return;
            if (!TryGetCollisionPose(out Vector3 bodyPos, out Quaternion bodyRot)) return;

            RebuildWorldCollisionCacheIfNeeded(bodyPos, bodyRot);

            int color = ColorUtil.ToRgba(255, 255, 0, 0);

            for (int i = 0; i < cachedWorldBoxes.Count; i++)
            {
                Cuboidd b = cachedWorldBoxes[i].Box;
                DrawAabb(capi, b, color);
            }
        }

        private void DrawAabb(ICoreClientAPI capi, Cuboidd b, int color)
        {
            Vector3[] corners = new Vector3[8];

            corners[0] = new Vector3((float)b.X1, (float)b.Y1, (float)b.Z1);
            corners[1] = new Vector3((float)b.X2, (float)b.Y1, (float)b.Z1);
            corners[2] = new Vector3((float)b.X2, (float)b.Y2, (float)b.Z1);
            corners[3] = new Vector3((float)b.X1, (float)b.Y2, (float)b.Z1);

            corners[4] = new Vector3((float)b.X1, (float)b.Y1, (float)b.Z2);
            corners[5] = new Vector3((float)b.X2, (float)b.Y1, (float)b.Z2);
            corners[6] = new Vector3((float)b.X2, (float)b.Y2, (float)b.Z2);
            corners[7] = new Vector3((float)b.X1, (float)b.Y2, (float)b.Z2);

            BlockPos origin = entity.Pos.AsBlockPos;

            for (int i = 0; i < 8; i++)
            {
                corners[i] -= new Vector3(origin.X, origin.Y, origin.Z);
            }

            Line(capi, origin, corners[0], corners[1], color);
            Line(capi, origin, corners[1], corners[2], color);
            Line(capi, origin, corners[2], corners[3], color);
            Line(capi, origin, corners[3], corners[0], color);

            Line(capi, origin, corners[4], corners[5], color);
            Line(capi, origin, corners[5], corners[6], color);
            Line(capi, origin, corners[6], corners[7], color);
            Line(capi, origin, corners[7], corners[4], color);

            Line(capi, origin, corners[0], corners[4], color);
            Line(capi, origin, corners[1], corners[5], color);
            Line(capi, origin, corners[2], corners[6], color);
            Line(capi, origin, corners[3], corners[7], color);
        }

        private void Line(ICoreClientAPI capi, BlockPos origin, Vector3 a, Vector3 b, int color)
        {
            capi.Render.RenderLine(
                origin,
                a.X, a.Y, a.Z,
                b.X, b.Y, b.Z,
                color
            );
        }

        private void RebuildWorldCollisionCacheIfNeeded(Vector3 bodyPosition, Quaternion bodyOrientation)
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

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                ManualChildBox child = manualChildBoxes[i];

                Quaternion childWorldOrientation = Quaternion.Normalize(bodyOrientation * child.LocalOrientation);
                Vector3 childWorldCenter = bodyPosition + Vector3.Transform(child.LocalPosition, bodyOrientation);

                Cuboidd worldAabb = CreateAabbFromOrientedBox(
                    childWorldCenter,
                    childWorldOrientation,
                    child.HalfExtents
                );

                cachedWorldBoxes.Add(new DynamicCollisionBox
                {
                    Box = worldAabb,
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

        private static Matrix4x4 CreateVsElementLocalMatrix(ShapeElement elem)
        {
            float ox = elem.RotationOrigin != null ? (float)elem.RotationOrigin[0] / 16f : 0f;
            float oy = elem.RotationOrigin != null ? (float)elem.RotationOrigin[1] / 16f : 0f;
            float oz = elem.RotationOrigin != null ? (float)elem.RotationOrigin[2] / 16f : 0f;

            float fx = elem.From != null ? (float)elem.From[0] / 16f : 0f;
            float fy = elem.From != null ? (float)elem.From[1] / 16f : 0f;
            float fz = elem.From != null ? (float)elem.From[2] / 16f : 0f;

            float sx = (float)elem.ScaleX;
            float sy = (float)elem.ScaleY;
            float sz = (float)elem.ScaleZ;

            Matrix4x4 translateToRotationOrigin = Matrix4x4.CreateTranslation(ox, oy, oz);
            Matrix4x4 rotateX = elem.RotationX != 0.0 ? Matrix4x4.CreateRotationX(DegreesToRadians((float)elem.RotationX)) : Matrix4x4.Identity;
            Matrix4x4 rotateY = elem.RotationY != 0.0 ? Matrix4x4.CreateRotationY(DegreesToRadians((float)elem.RotationY)) : Matrix4x4.Identity;
            Matrix4x4 rotateZ = elem.RotationZ != 0.0 ? Matrix4x4.CreateRotationZ(DegreesToRadians((float)elem.RotationZ)) : Matrix4x4.Identity;
            Matrix4x4 scale = (sx != 1f || sy != 1f || sz != 1f) ? Matrix4x4.CreateScale(sx, sy, sz) : Matrix4x4.Identity;

            Matrix4x4 translateFromOriginToElementFrom = Matrix4x4.CreateTranslation(fx - ox, fy - oy, fz - oz);

            return translateFromOriginToElementFrom
                * scale
                * rotateZ
                * rotateY
                * rotateX
                * translateToRotationOrigin;
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

            Matrix4x4 local = CreateVsElementLocalMatrix(elem);
            Matrix4x4 world = local * parentWorld;

            if (selected && elem.From != null && elem.To != null)
            {
                float sx = ((float)elem.To[0] - (float)elem.From[0]) / 16f;
                float sy = ((float)elem.To[1] - (float)elem.From[1]) / 16f;
                float sz = ((float)elem.To[2] - (float)elem.From[2]) / 16f;

                if (sx > 0f && sy > 0f && sz > 0f)
                {
                    Vector3 localCenter = new Vector3(sx * 0.5f, sy * 0.5f, sz * 0.5f);
                    Vector3 childPosition = TransformPoint(world, localCenter);

                    Quaternion childOrientation = ExtractPureRotation(world);
                    Vector3 axisScale = ExtractAxisScale(world);

                    float width = MathF.Abs(sx * axisScale.X);
                    float height = MathF.Abs(sy * axisScale.Y);
                    float length = MathF.Abs(sz * axisScale.Z);

                    if (width > 1e-5f && height > 1e-5f && length > 1e-5f)
                    {
                        manualBoxes.Add(new ManualChildBox
                        {
                            LocalPosition = childPosition,
                            LocalOrientation = childOrientation,
                            HalfExtents = new Vector3(width * 0.5f, height * 0.5f, length * 0.5f)
                        });

                        float mass = width * height * length;
                        childMasses.Add(mass);
                    }
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

                if (WildcardMatch(path, selector))
                    return true;
            }

            return false;
        }

        private static bool WildcardMatch(string text, string pattern)
        {
            int t = 0;
            int p = 0;
            int star = -1;
            int match = 0;

            while (t < text.Length)
            {
                if (p < pattern.Length && (pattern[p] == '?' || pattern[p] == text[t]))
                {
                    t++;
                    p++;
                    continue;
                }

                if (p < pattern.Length && pattern[p] == '*')
                {
                    star = p++;
                    match = t;
                    continue;
                }

                if (star != -1)
                {
                    p = star + 1;
                    t = ++match;
                    continue;
                }

                return false;
            }

            while (p < pattern.Length && pattern[p] == '*')
                p++;

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

        private bool TryGetCollisionPose(out Vector3 bodyPosition, out Quaternion bodyOrientation)
        {
            bodyPosition = Vector3.Zero;
            bodyOrientation = Quaternion.Identity;

            if (entity?.Pos == null)
                return false;

            EntityPos pos = entity.Pos;

            Quaternion entityRotation = Quaternion.CreateFromYawPitchRoll(
                pos.Yaw,
                pos.Pitch,
                pos.Roll
            );

            Quaternion correction = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2f);

            bodyOrientation = Quaternion.Normalize(correction * entityRotation);

            Vector3 entityOrigin = ToVector3(pos.X, pos.Y, pos.Z);
            Vector3 localAnchorCorrection = new Vector3(-0.5f, 0f, -0.5f);

            bodyPosition =
                entityOrigin +
                Vector3.Transform(localAnchorCorrection + localCenterOfMassOffset, bodyOrientation);

            return true;
        }

        private static Cuboidd CreateAabbFromOrientedBox(
            Vector3 center,
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
                center.X - extents.X,
                center.Y - extents.Y,
                center.Z - extents.Z,
                center.X + extents.X,
                center.Y + extents.Y,
                center.Z + extents.Z
            );
        }
    }
}