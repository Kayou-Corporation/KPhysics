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

    ColliderID AddSphereCollider(const BodyID& bodyID, const Vec3& positionOffset, const Quat& rotationOffset, float radius);
    ColliderID AddBoxCollider(const BodyID& bodyID, const Vec3& positionOffset, const Quat& rotationOffset, const Vec3& halfExtents);
    ColliderID AddCapsuleCollider(const BodyID& bodyID, const Vec3& positionOffset, const Quat& rotationOffset, float radius, float halfHeight);
    void DestroyCollider(ColliderID& colliderID); // Invalidates the ColliderID passed as reference

    void AddLinearImpulse(const BodyID& bodyID, const Vec3& impulse);
    void AddAngularImpulse(const BodyID& bodyID, const Vec3& impulse);
    void AddForce(const BodyID& bodyID, const Vec3& force);
    void AddTorque(const BodyID& bodyID, const Vec3& torque);

    KNO_DISCARD Vec3 GetBodyPosition(const BodyID& bodyID) const { return IsValid(bodyID) ? m_bodyStorage.transforms.positions[bodyID.Index()] : Vec3(); }

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

    void DestroySphere(const ColliderID& colliderID);
    void DestroyBox(const ColliderID& colliderID);
    void DestroyCapsule(const ColliderID& colliderID);

    KNO_DISCARD bool IsValid(const BodyID& bodyID) const;
    KNO_DISCARD bool IsValid(const ColliderID& colliderID) const;

};

END_NAMESPACE_KAYOU