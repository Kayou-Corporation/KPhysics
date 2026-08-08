#pragma once

#include <cstdint>

#include "Utils.hpp"
#include "BodyStorage.hpp"
#include "ColliderStorage.hpp"
#include "Preprocessor.hpp"

BEGIN_NAMESPACE_KAYOU

class PhysicsWorld
{
public:
    explicit PhysicsWorld(uint32_t preallocatedBodies = 1024, float gravity = 9.81f);

    void Step(float dt);
    BodyID CreateBody(const Vec3& position, const Quat& rotation, float mass);
    void DestroyBody(BodyID& bodyID); // Invalidates the BodyID passed as reference
    ColliderID AddSphereCollider(BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset, float radius);
    ColliderID AddBoxCollider(BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset, const Vec3& halfExtents);
    ColliderID AddCapsuleCollider(BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset, float radius, float halfHeight);

    void AddLinearImpulse(BodyID bodyID, const Vec3& impulse);
    void AddAngularImpulse(BodyID bodyID, const Vec3& impulse);
    void AddForce(BodyID bodyID, const Vec3& force);
    void AddTorque(BodyID bodyID, const Vec3& torque);

    KNO_DISCARD Vec3 GetBodyPosition(const BodyID bodyID) const { return m_bodyStorage.transforms.positions[bodyID.Index()]; }

private:
    BodyStorage m_bodyStorage;
    ColliderStorage m_colliderStorage;
    float m_gravity = 0.0f;

    void UpdateWorldInertia();
    void Gravity();
    void IntegrateVelocities(float dt);
    void IntegratePositions(float dt);
    void IntegrateRotations(float dt);
    void ClearForces();
    void ClearTorques();

    KNO_DISCARD ColliderID AddCollider(BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset);

};

END_NAMESPACE_KAYOU