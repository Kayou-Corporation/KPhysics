#pragma once

#include <cstdint>

#include "Utils.hpp"
#include "BodyStorage.hpp"

BEGIN_NAMESPACE_KAYOU

using BodyID = uint32_t;

class PhysicsWorld
{
public:
    explicit PhysicsWorld(uint32_t preallocatedBodies = 1024, float gravity = 9.81f);

    void Step(float dt);
    BodyID CreateBody(const Vec3& position, const Quat& rotation, float mass);

    Vec3 GetBodyPosition(BodyID bodyID);

private:
    BodyStorage m_bodyStorage;
    float m_gravity = 0.0f;

    void Gravity();
    void IntegrateVelocities(float dt);
    void IntegratePositions(float dt);
    void ClearForces();

};

END_NAMESPACE_KAYOU