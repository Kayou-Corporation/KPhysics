#pragma once

#include "Maths/Mat3.hpp"
#include "Maths/Quaternion.hpp"
#include "Maths/Vec3.hpp"

BEGIN_NAMESPACE_KAYOU

struct TransformStorage
{
    Array<Vec3> positions;
    Array<Quat> rotations;

    explicit TransformStorage(uint32_t capacity)
    {
        positions.reserve(capacity);
        rotations.reserve(capacity);
    }

    ~TransformStorage()
    {
        positions.clear();
        rotations.clear();
    }
};

struct VelocityStorage
{
    Array<Vec3> linear;
    Array<Vec3> angular;

    explicit VelocityStorage(uint32_t capacity)
    {
        linear.reserve(capacity);
        angular.reserve(capacity);
    }

    ~VelocityStorage()
    {
        linear.clear();
        angular.clear();
    }
};

struct MassStorage
{
    Array<float> inverseMasses;
    Array<Mat3> inverseInertias;
    Array<Mat3> inverseInertiasWorld;

    explicit MassStorage(uint32_t capacity)
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
};

struct ForceStorage
{
    Array<Vec3> forces;
    Array<Vec3> torques;

    explicit ForceStorage(uint32_t capacity)
    {
        forces.reserve(capacity);
        torques.reserve(capacity);
    }

    ~ForceStorage()
    {
        forces.clear();
        torques.clear();
    }
};

class BodyStorage
{
public:
    explicit BodyStorage(uint32_t capacity)
        : transforms(capacity), velocities(capacity), masses(capacity), forces(capacity), capacity{capacity} {}

    TransformStorage transforms;
    VelocityStorage velocities;
    MassStorage masses;
    ForceStorage forces;

    uint32_t capacity;

    //Array<BodyFlags> flags;
    //Array<ColliderID> colliders;
};

END_NAMESPACE_KAYOU