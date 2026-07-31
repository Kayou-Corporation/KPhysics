#pragma once

#include "KPhysics/Utils.hpp"
#include "Vec3.hpp"
#include "Scalar.hpp"

BEGIN_NAMESPACE_KAYOU

struct AABB
{
    AABB() = default;
    AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}

    Vec3 min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    Vec3 max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
};

inline AABB ComputeAABB(const Vec3& pos, const Vec3& scale)
{
    AABB box;
    box.min = pos - (0.5f * scale);
    box.max = pos + (0.5f * scale);
    return box;
}

inline AABB ComputeAABB(const std::vector<Vec3>& points)
{
    AABB box;
    for (const Vec3& point : points)
    {
        box.min = Min(box.min, point);
        box.max = Max(box.max, point);
    }
    return box;
}

inline AABB MergeBoxes(const AABB& box1, const AABB& box2)
{
    AABB box;
    box.min = Min(box1.min, box2.min);
    box.max = Max(box1.max, box2.max);
    return box;
}

inline Vec3 GetBoxCenter(const AABB& box)
{
    return box.min + (box.max - box.min) * 0.5f;
}

inline Vec3 GetBoxSize(const AABB& box)
{
    return box.max - box.min;
}

inline Vec3 GetBoxHalfExtents(const AABB& box)
{
    return GetBoxSize(box) * 0.5f;
}

inline float GetBoxVolume(const AABB& box)
{
    const Vec3 minPos = box.min;
    const Vec3 maxPos = box.max;

    const float dx = maxPos.x - minPos.x;
    const float dy = maxPos.y - minPos.y;
    const float dz = maxPos.z - minPos.z;

    return dx * dy * dz;
}

inline float BoxSurfaceArea(const AABB& box)
{
    const Vec3 size = box.max - box.min;

    return 2.0f * (
        size.x * size.y +
        size.x * size.z +
        size.y * size.z);
}

inline float GetBoxUnionCost(const AABB& box1, const AABB& box2)
{
    const float unionArea = BoxSurfaceArea(MergeBoxes(box1, box2));
    const float cost = unionArea - BoxSurfaceArea(box1);
    return cost;
}

inline bool CheckBoxIntersection(const AABB& box1, const AABB& box2)
{
    return
        box1.min.x <= box2.max.x &&
        box1.max.x >= box2.min.x &&

        box1.min.y <= box2.max.y &&
        box1.max.y >= box2.min.y &&

        box1.min.z <= box2.max.z &&
        box1.max.z >= box2.min.z;
}

inline bool BoxContains(const AABB& box, const Vec3& point)
{
    return box.min.x <= point.x && point.x <= box.max.x && box.min.y <= point.y && point.y <= box.max.y && box.min.z <= point.z && point.z <= box.max.z;
}

inline bool BoxContains(const AABB& bigBox, const AABB& box)
{
    return BoxContains(bigBox, box.min) && BoxContains(bigBox, box.max);
}

inline AABB ExpandBox(const AABB& box, const float amount)
{
    return AABB(box.min - amount, box.max + amount);
}

inline Vec3 ClosestPointFromBox(const AABB& box, const Vec3& point)
{
    return Vec3(Clamp(point.x, box.min.x, box.max.x), Clamp(point.y, box.min.y, box.max.y), Clamp(point.z, box.min.z, box.max.z));
}

inline float DistanceFromBoxSquared(const AABB& box, const Vec3& point)
{
    const Vec3 closestPoint = ClosestPointFromBox(box, point);
    return LengthSquared(closestPoint - point);
}

inline bool IsBoxValid(const AABB& box)
{
    return
        box.min.x <= box.max.x &&
        box.min.y <= box.max.y &&
        box.min.z <= box.max.z;
}

END_NAMESPACE_KAYOU