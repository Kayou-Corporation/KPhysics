#pragma once

#include <queue>

#include "BodyID.hpp"
#include "Maths/Mat3.hpp"
#include "Maths/Quaternion.hpp"
#include "Maths/Vec3.hpp"

BEGIN_NAMESPACE_KAYOU

struct TransformStorage
{
    explicit TransformStorage(const uint32_t capacity)
    {
        positions.reserve(capacity);
        rotations.reserve(capacity);
    }

    ~TransformStorage()
    {
        positions.clear();
        rotations.clear();
    }

    Array<Vec3> positions;
    Array<Quat> rotations;
};

struct VelocityStorage
{
    explicit VelocityStorage(const uint32_t capacity)
    {
        linear.reserve(capacity);
        angular.reserve(capacity);
    }

    ~VelocityStorage()
    {
        linear.clear();
        angular.clear();
    }

    Array<Vec3> linear;
    Array<Vec3> angular;
};

struct MassStorage
{
    explicit MassStorage(const uint32_t capacity)
    {
        inverseMasses.reserve(capacity);
        inverseInertias.reserve(capacity);
        inverseInertiasWorld.reserve(capacity);
    }

    ~MassStorage()
    {
        inverseMasses.clear();
        inverseInertias.clear();
        inverseInertiasWorld.clear();
    }

    Array<float> inverseMasses;
    Array<Mat3> inverseInertias;
    Array<Mat3> inverseInertiasWorld;
};

struct ForceStorage
{
    explicit ForceStorage(const uint32_t capacity)
    {
        forces.reserve(capacity);
        torques.reserve(capacity);
    }

    ~ForceStorage()
    {
        forces.clear();
        torques.clear();
    }

    Array<Vec3> forces;
    Array<Vec3> torques;
};

class BodyStorage
{
public:
    explicit BodyStorage(const uint32_t capacity)
        : transforms(capacity), velocities(capacity), masses(capacity), forces(capacity), capacity{capacity}
    {
        colliders.reserve(capacity);
        generations.reserve(capacity);
    }

    ~BodyStorage()
    {
        colliders.clear();
    }

    TransformStorage transforms;
    VelocityStorage velocities;
    MassStorage masses;
    ForceStorage forces;

    Array<ColliderID> colliders;

    uint32_t capacity;

    std::queue<uint32_t> availableIndices;
    Array<uint8_t> generations;

    //Array<BodyFlags> flags;
    //Array<ColliderID> colliders;
};

END_NAMESPACE_KAYOU