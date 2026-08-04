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
    const BodyID id = static_cast<BodyID>(m_bodyStorage.transforms.positions.size());

    m_bodyStorage.transforms.positions.push_back(position);
    m_bodyStorage.transforms.rotations.push_back(rotation);

    m_bodyStorage.velocities.linear.push_back(Vec3(0.0f));
    m_bodyStorage.velocities.angular.push_back(Vec3(0.0f));

    const float invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    m_bodyStorage.masses.inverseMasses.push_back(invMass);
    m_bodyStorage.masses.inverseInertias.push_back(Mat3(1.f));

    m_bodyStorage.forces.forces.push_back(Vec3(0.0f));
    m_bodyStorage.forces.torques.push_back(Vec3(0.0f));

    return id;
}

void PhysicsWorld::AddLinearImpulse(const BodyID id, const Vec3& impulse)
{
    const float invMass = m_bodyStorage.masses.inverseMasses[id];
    if (invMass <= 0.0f)
        return;

    m_bodyStorage.velocities.linear[id] += impulse * invMass;
}

void PhysicsWorld::AddAngularImpulse(const BodyID id, const Vec3& impulse)
{
    if (m_bodyStorage.masses.inverseMasses[id] <= 0.0f)
        return;

    const Mat3& invInertiaWorld = m_bodyStorage.masses.inverseInertiasWorld[id];
    m_bodyStorage.velocities.angular[id] += invInertiaWorld * impulse;
}

void PhysicsWorld::AddTorque(const BodyID id, const Vec3& torque)
{
    if (m_bodyStorage.masses.inverseMasses[id] <= 0.0f)
        return;

    m_bodyStorage.forces.torques[id] += torque;
}

void PhysicsWorld::AddForce(const BodyID id, const Vec3& force)
{
    if (m_bodyStorage.masses.inverseMasses[id] <= 0.0f)
        return;

    m_bodyStorage.forces.forces[id] += force;
}

void PhysicsWorld::UpdateWorldInertia()
{
    for (size_t i = 0; i < m_bodyStorage.masses.inverseInertias.size(); ++i)
    {
        Mat3 rotation = ToMat3(m_bodyStorage.transforms.rotations[i]);
        m_bodyStorage.masses.inverseInertiasWorld[i] = rotation * m_bodyStorage.masses.inverseInertias[i] * Transpose(rotation);
    }
}

void PhysicsWorld::Gravity()
{
    const Vec3 gravityAcceleration = Vec3(0.0f, -m_gravity, 0.0f);

    for (size_t i = 0; i < m_bodyStorage.forces.forces.size(); ++i)
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
    for (size_t i = 0; i < m_bodyStorage.velocities.linear.size(); ++i)
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
    for (size_t i = 0; i < m_bodyStorage.transforms.positions.size(); ++i)
    {
        m_bodyStorage.transforms.positions[i] += m_bodyStorage.velocities.linear[i] * dt;
    }
}

void PhysicsWorld::IntegrateRotations(const float dt)
{
    for (size_t i = 0; i < m_bodyStorage.transforms.rotations.size(); ++i)
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
    for (size_t i = 0; i < m_bodyStorage.forces.forces.size(); ++i)
    {
        m_bodyStorage.forces.forces[i] = Vec3(0.0f);
    }
}

void PhysicsWorld::ClearTorques()
{
    for (size_t i = 0; i < m_bodyStorage.forces.torques.size(); ++i)
    {
        m_bodyStorage.forces.torques[i] = Vec3(0.0f);
    }
}

END_NAMESPACE_KAYOU
