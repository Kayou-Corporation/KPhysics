#pragma once
#include <cstdint>

#include "KPhysics.hpp"
#include "Maths/Quaternion.hpp"
#include "Maths/Vec3.hpp"

BEGIN_NAMESPACE_KAYOU

enum class ShapeType : uint8_t
{
    SPHERE = 0,
    BOX,
    CAPSULE,
    NONE
};

struct ShapeReference
{
    ShapeType type;
    ShapeID index;
};

struct SphereStorage
{
    explicit SphereStorage(const uint32_t capacity)
    {
        radii.reserve(capacity);
    }

    ~SphereStorage()
    {
        radii.clear();
    }

    Array<float> radii;
};

struct BoxStorage
{
    explicit BoxStorage(const uint32_t capacity)
    {
        halfExtents.reserve(capacity);
    }

    ~BoxStorage()
    {
        halfExtents.clear();
    }

    Array<Vec3> halfExtents;
};

struct CapsuleStorage
{
    explicit CapsuleStorage(const uint32_t capacity)
    {
        radii.reserve(capacity);
        halfHeights.reserve(capacity);
    }

    ~CapsuleStorage()
    {
        radii.clear();
        halfHeights.clear();
    }

    Array<float> radii;
    Array<float> halfHeights;
};

class ColliderStorage
{
public:
    explicit ColliderStorage(const uint32_t capacity)
        : spheres(capacity), boxes(capacity), capsules(capacity)
    {
        bodies.reserve(capacity);
        localPositions.reserve(capacity);
        localRotations.reserve(capacity);
        shapes.reserve(capacity);
    }

    ~ColliderStorage()
    {
        bodies.clear();
        localPositions.clear();
        localRotations.clear();
        shapes.clear();
    }

    Array<BodyID> bodies;

    Array<Vec3> localPositions;
    Array<Quat> localRotations;

    Array<ShapeReference> shapes;

    SphereStorage spheres;
    BoxStorage boxes;
    CapsuleStorage capsules;

    //Array<MaterialID> materials; later
};

END_NAMESPACE_KAYOU
