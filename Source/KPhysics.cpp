#include "KPhysics/KPhysics.hpp"

BEGIN_NAMESPACE_KAYOU

PhysicsWorld::PhysicsWorld(uint32_t preallocatedBodies, float gravity) : m_bodyStorage(preallocatedBodies), m_gravity(gravity)
{

}

void PhysicsWorld::Step(float dt)
{
    Gravity();
    IntegrateVelocities(dt);
    IntegratePositions(dt);
    ClearForces();
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

Vec3 PhysicsWorld::GetBodyPosition(BodyID bodyID)
{
    return m_bodyStorage.transforms.positions[bodyID];
}

void PhysicsWorld::Gravity()
{
    const Vec3 gravityAcceleration = Vec3(0.0f, -m_gravity, 0.0f);

    for (size_t i = 0; i < m_bodyStorage.transforms.positions.size(); ++i)
    {
        const float invMass = m_bodyStorage.masses.inverseMasses[i];
        if (invMass == 0.0f)
            continue;

        const float mass = 1.0f / invMass;
        m_bodyStorage.forces.forces[i] += gravityAcceleration * mass;
    }
}

void PhysicsWorld::IntegrateVelocities(const float dt)
{
    for (size_t i = 0; i < m_bodyStorage.transforms.positions.size(); ++i)
    {
        Vec3 acceleration = m_bodyStorage.forces.forces[i] * m_bodyStorage.masses.inverseMasses[i];
        m_bodyStorage.velocities.linear[i] += acceleration * dt;
    }
}

void PhysicsWorld::IntegratePositions(const float dt)
{
    for (size_t i = 0; i < m_bodyStorage.transforms.positions.size(); ++i)
    {
        m_bodyStorage.transforms.positions[i] += m_bodyStorage.velocities.linear[i] * dt;
    }
}

void PhysicsWorld::ClearForces()
{
    for (size_t i = 0; i < m_bodyStorage.transforms.positions.size(); ++i)
    {
        m_bodyStorage.forces.forces[i] = Vec3(0.0f);
    }
}

END_NAMESPACE_KAYOU
