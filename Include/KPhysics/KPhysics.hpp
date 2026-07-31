#pragma once

#include <cstdint>

#include "Utils.hpp"
#include "BodyStorage.hpp"
#include "Preprocessor.hpp"

BEGIN_NAMESPACE_KAYOU

using BodyID = uint32_t;

enum class ShapeType
{
    SPHERE,
    BOX,
    NONE
};

class PhysicsWorld
{
public:
    explicit PhysicsWorld(uint32_t preallocatedBodies = 1024, float gravity = 9.81f);

    void Step(float dt);
    BodyID CreateBody(const Vec3& position, const Quat& rotation, float mass);

    void AddLinearImpulse(BodyID id, const Vec3& impulse);
    void AddAngularImpulse(BodyID id, const Vec3& impulse);
    void AddForce(BodyID id, const Vec3& force);
    void AddTorque(BodyID id, const Vec3& torque);

    KNO_DISCARD Vec3 GetBodyPosition(const BodyID bodyID) const { return m_bodyStorage.transforms.positions[bodyID]; }

private:
    BodyStorage m_bodyStorage;
    float m_gravity = 0.0f;

    void UpdateWorldInertia();
    void Gravity();
    void IntegrateVelocities(float dt);
    void IntegratePositions(float dt);
    void ClearForces();
    void ClearTorques();

};

END_NAMESPACE_KAYOU