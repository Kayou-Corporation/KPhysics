#include "KPhysics/KPhysics.hpp"

BEGIN_NAMESPACE_KAYOU

PhysicsWorld::PhysicsWorld(uint32_t preallocatedBodies, const float gravity) : m_bodyStorage(preallocatedBodies),
                                                                         m_colliderStorage(preallocatedBodies),
                                                                         m_gravity(gravity)
{

}

void PhysicsWorld::Step(float dt)
{
    Gravity();

    IntegrateVelocities(dt);

    IntegratePositions(dt);
    IntegrateRotations(dt);

    UpdateWorldInertia();

    ClearForces();
    ClearTorques();
}

BodyID PhysicsWorld::CreateBody(const Vec3& position, const Quat& rotation, float mass)
{
    uint32_t index;

    if (!m_bodyStorage.availableIndices.empty())
    {
        index = m_bodyStorage.availableIndices.front();
        m_bodyStorage.availableIndices.pop();
    }
    else
    {
        index = static_cast<uint32_t>(m_bodyStorage.transforms.positions.size());

        if (index >= BodyID::MAX_BODY_ID)
            return BodyID(BodyID::INVALID);

        m_bodyStorage.transforms.positions.emplace_back();
        m_bodyStorage.transforms.rotations.emplace_back();

        m_bodyStorage.velocities.linear.emplace_back();
        m_bodyStorage.velocities.angular.emplace_back();

        m_bodyStorage.masses.inverseMasses.emplace_back();
        m_bodyStorage.masses.inverseInertias.emplace_back();

        m_bodyStorage.forces.forces.emplace_back();
        m_bodyStorage.forces.torques.emplace_back();

        m_bodyStorage.generations.push_back(0);
    }

    m_bodyStorage.transforms.positions[index] = position;
    m_bodyStorage.transforms.rotations[index] = rotation;

    m_bodyStorage.velocities.linear[index] = Vec3(0.0f);
    m_bodyStorage.velocities.angular[index] = Vec3(0.0f);

    const float invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    m_bodyStorage.masses.inverseMasses[index] = invMass;
    m_bodyStorage.masses.inverseInertias[index] = Mat3(1.0f);

    m_bodyStorage.forces.forces[index] = Vec3(0.0f);
    m_bodyStorage.forces.torques[index] = Vec3(0.0f);

    return BodyID(index, m_bodyStorage.generations[index]);
}

void PhysicsWorld::DestroyBody(BodyID& bodyID)
{
    const uint32_t index = bodyID.Index();
    const uint8_t generation = bodyID.Generation();

    if (m_bodyStorage.generations[index] != generation)
        return;

    m_bodyStorage.availableIndices.push(index);
    m_bodyStorage.generations[index] = (generation + 1) & 0xFF;

    bodyID = BodyID(BodyID::INVALID);
}

void PhysicsWorld::AddSphereCollider(const BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset,
                                     const float radius)
{
    AddCollider(bodyID, positionOffset, rotationOffset);

    m_colliderStorage.spheres.radii.push_back(radius);

    const ShapeID sphereID = static_cast<ShapeID>(m_colliderStorage.spheres.radii.size() - 1);
    m_colliderStorage.shapes.emplace_back(ShapeType::SPHERE, sphereID);

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass <= 0.0f)
        return;

    m_bodyStorage.masses.inverseInertias[bodyID.Index()] = ComputeSphereInverseInertia(invMass, radius);
}

void PhysicsWorld::AddBoxCollider(const BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset,
    const Vec3& halfExtents)
{
    AddCollider(bodyID, positionOffset, rotationOffset);

    m_colliderStorage.boxes.halfExtents.push_back(halfExtents);

    const ShapeID boxID = static_cast<ShapeID>(m_colliderStorage.boxes.halfExtents.size() - 1);
    m_colliderStorage.shapes.emplace_back(ShapeType::BOX, boxID);

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass <= 0.0f)
        return;

    m_bodyStorage.masses.inverseInertias[bodyID.Index()] = ComputeBoxInverseInertia(invMass, halfExtents);
}

void PhysicsWorld::AddCapsuleCollider(const BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset,
    const float radius, const float halfHeight)
{
    AddCollider(bodyID, positionOffset, rotationOffset);

    m_colliderStorage.capsules.radii.push_back(radius);
    m_colliderStorage.capsules.halfHeights.push_back(halfHeight);

    const ShapeID capsuleID = static_cast<ShapeID>(m_colliderStorage.capsules.radii.size() - 1);
    m_colliderStorage.shapes.emplace_back(ShapeType::CAPSULE, capsuleID);

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass <= 0.0f)
        return;

    m_bodyStorage.masses.inverseInertias[bodyID.Index()] = ComputeCapsuleInverseInertia(invMass, radius, halfHeight);
}

void PhysicsWorld::AddLinearImpulse(const BodyID bodyID, const Vec3& impulse)
{
    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass <= 0.0f)
        return;

    m_bodyStorage.velocities.linear[bodyID.Index()] += impulse * invMass;
}

void PhysicsWorld::AddAngularImpulse(const BodyID bodyID, const Vec3& impulse)
{
    if (m_bodyStorage.masses.inverseMasses[bodyID.Index()] <= 0.0f)
        return;

    const Mat3& invInertiaWorld = m_bodyStorage.masses.inverseInertiasWorld[bodyID.Index()];
    m_bodyStorage.velocities.angular[bodyID.Index()] += invInertiaWorld * impulse;
}

void PhysicsWorld::AddTorque(const BodyID bodyID, const Vec3& torque)
{
    if (m_bodyStorage.masses.inverseMasses[bodyID.Index()] <= 0.0f)
        return;

    m_bodyStorage.forces.torques[bodyID.Index()] += torque;
}

void PhysicsWorld::AddForce(const BodyID bodyID, const Vec3& force)
{
    if (m_bodyStorage.masses.inverseMasses[bodyID.Index()] <= 0.0f)
        return;

    m_bodyStorage.forces.forces[bodyID.Index()] += force;
}

void PhysicsWorld::UpdateWorldInertia()
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.masses.inverseInertias.size()); ++i)
    {
        if (m_bodyStorage.masses.inverseMasses[i] <= 0.0f)
            return;

        Mat3 rotation = ToMat3(m_bodyStorage.transforms.rotations[i]);
        m_bodyStorage.masses.inverseInertiasWorld[i] = rotation * m_bodyStorage.masses.inverseInertias[i] * Transpose(rotation);
    }
}

void PhysicsWorld::Gravity()
{
    const Vec3 gravityAcceleration = Vec3(0.0f, -m_gravity, 0.0f);

    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.forces.forces.size()); ++i)
    {
        const float invMass = m_bodyStorage.masses.inverseMasses[i];
        if (invMass <= 0.0f)
            continue;

        const float mass = 1.0f / invMass;
        m_bodyStorage.forces.forces[i] += gravityAcceleration * mass;
    }
}

void PhysicsWorld::IntegrateVelocities(const float dt)
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.velocities.linear.size()); ++i)
    {
        const float invMass = m_bodyStorage.masses.inverseMasses[i];
        if (invMass <= 0.0f)
            continue;

        Vec3 linearAcceleration = m_bodyStorage.forces.forces[i] * invMass;
        m_bodyStorage.velocities.linear[i] += linearAcceleration * dt;

        Vec3 angularAcceleration = m_bodyStorage.masses.inverseInertiasWorld[i] * m_bodyStorage.forces.torques[i];
        m_bodyStorage.velocities.angular[i] += angularAcceleration * dt;
    }
}

void PhysicsWorld::IntegratePositions(const float dt)
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.transforms.positions.size()); ++i)
    {
        m_bodyStorage.transforms.positions[i] += m_bodyStorage.velocities.linear[i] * dt;
    }
}

void PhysicsWorld::IntegrateRotations(const float dt)
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.transforms.rotations.size()); ++i)
    {
        const Vec3& angularVelocity = m_bodyStorage.velocities.angular[i];
        if (LengthSquared(angularVelocity) <= 0.f)
            continue;

        Quat w = {0.f, angularVelocity.x, angularVelocity.y, angularVelocity.z};
        Quat& q = m_bodyStorage.transforms.rotations[i];

        q = 0.5f * (w * q) * dt;
        q = Normalize(q);
    }
}

void PhysicsWorld::ClearForces()
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.forces.forces.size()); ++i)
    {
        m_bodyStorage.forces.forces[i] = Vec3(0.0f);
    }
}

void PhysicsWorld::ClearTorques()
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.forces.torques.size()); ++i)
    {
        m_bodyStorage.forces.torques[i] = Vec3(0.0f);
    }
}

void PhysicsWorld::AddCollider(const BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset)
{
    m_colliderStorage.bodies.push_back(bodyID);
    m_colliderStorage.localPositions.push_back(positionOffset);
    m_colliderStorage.localRotations.push_back(rotationOffset);
}

END_NAMESPACE_KAYOU
