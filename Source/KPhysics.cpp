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
        m_bodyStorage.alive.emplace_back();
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

    m_bodyStorage.alive[index] = true;

    return BodyID(index, m_bodyStorage.generations[index]);
}

void PhysicsWorld::DestroyBody(BodyID& bodyID)
{
    if (!IsValid(bodyID))
        return;

    const uint32_t index = bodyID.Index();
    const uint8_t generation = bodyID.Generation();

    const Array<ColliderID>& colliders = m_bodyStorage.colliders[index];

    while (!colliders.empty())
    {
        ColliderID colliderID = colliders.back();
        DestroyCollider(colliderID);
    }

    m_bodyStorage.availableIndices.push(index);
    m_bodyStorage.generations[index] = (generation + 1) & 0xFF;
    m_bodyStorage.alive[index] = false;

    bodyID = BodyID(BodyID::INVALID);
}

ColliderID PhysicsWorld::AddSphereCollider(const BodyID& bodyID, const Vec3& positionOffset, const Quat& rotationOffset,
                                     const float radius)
{
    if (!IsValid(bodyID))
        return ColliderID(ColliderID::INVALID);

    ColliderID colliderID = AddCollider(bodyID, positionOffset, rotationOffset);

    if (!colliderID)
        return colliderID;

    uint32_t sphereIndex;

    if (!m_colliderStorage.availableIndices.empty())
    {
        sphereIndex = m_colliderStorage.spheres.availableIndices.front();
        m_colliderStorage.spheres.availableIndices.pop();
    }
    else
    {
        sphereIndex = static_cast<uint32_t>(m_colliderStorage.spheres.radii.size());

        if (sphereIndex >= ShapeID::MAX_BODY_ID)
        {
            DestroyCollider(colliderID);
            return ColliderID(ColliderID::INVALID);
        }

        m_colliderStorage.spheres.radii.emplace_back();

        m_colliderStorage.spheres.generations.push_back(0);
    }

    m_colliderStorage.spheres.radii[sphereIndex] = radius;

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass > 0.0f)
        m_bodyStorage.masses.inverseInertias[bodyID.Index()] = ComputeSphereInverseInertia(invMass, radius);

    m_colliderStorage.shapes[colliderID.Index()] = ShapeReference(ShapeType::SPHERE, ShapeID(sphereIndex, m_colliderStorage.spheres.generations[sphereIndex]));

    return colliderID;
}

ColliderID PhysicsWorld::AddBoxCollider(const BodyID& bodyID, const Vec3& positionOffset, const Quat& rotationOffset,
    const Vec3& halfExtents)
{
    if (!IsValid(bodyID))
        return ColliderID(ColliderID::INVALID);

    ColliderID colliderID = AddCollider(bodyID, positionOffset, rotationOffset);

    if (!colliderID)
        return colliderID;

    uint32_t boxIndex;

    if (!m_colliderStorage.availableIndices.empty())
    {
        boxIndex = m_colliderStorage.boxes.availableIndices.front();
        m_colliderStorage.boxes.availableIndices.pop();
    }
    else
    {
        boxIndex = static_cast<uint32_t>(m_colliderStorage.boxes.halfExtents.size());

        if (boxIndex >= ShapeID::MAX_BODY_ID)
        {
            DestroyCollider(colliderID);
            return ColliderID(ColliderID::INVALID);
        }

        m_colliderStorage.boxes.halfExtents.emplace_back();
        m_colliderStorage.shapes.emplace_back();

        m_colliderStorage.boxes.generations.push_back(0);
    }

    m_colliderStorage.boxes.halfExtents[boxIndex] = halfExtents;

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass > 0.0f)
        m_bodyStorage.masses.inverseInertias[bodyID.Index()] = ComputeBoxInverseInertia(invMass, halfExtents);

    m_colliderStorage.shapes[colliderID.Index()] = ShapeReference(ShapeType::BOX, ShapeID(boxIndex, m_colliderStorage.boxes.generations[boxIndex]));

    return colliderID;
}

ColliderID PhysicsWorld::AddCapsuleCollider(const BodyID& bodyID, const Vec3& positionOffset, const Quat& rotationOffset,
    const float radius, const float halfHeight)
{
    if (!IsValid(bodyID))
        return ColliderID(ColliderID::INVALID);

    ColliderID colliderID = AddCollider(bodyID, positionOffset, rotationOffset);

    if (!colliderID)
        return colliderID;

    uint32_t capsuleIndex;

    if (!m_colliderStorage.availableIndices.empty())
    {
        capsuleIndex = m_colliderStorage.capsules.availableIndices.front();
        m_colliderStorage.capsules.availableIndices.pop();
    }
    else
    {
        capsuleIndex = static_cast<uint32_t>(m_colliderStorage.capsules.radii.size());

        if (capsuleIndex >= ShapeID::MAX_BODY_ID)
        {
            DestroyCollider(colliderID);
            return ColliderID(ColliderID::INVALID);
        }

        m_colliderStorage.capsules.radii.emplace_back();
        m_colliderStorage.capsules.halfHeights.emplace_back();
        m_colliderStorage.shapes.emplace_back();

        m_colliderStorage.capsules.generations.push_back(0);
    }

    m_colliderStorage.capsules.radii[capsuleIndex] = radius;
    m_colliderStorage.capsules.halfHeights[capsuleIndex] = halfHeight;

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass > 0.0f)
        m_bodyStorage.masses.inverseInertias[bodyID.Index()] = ComputeCapsuleInverseInertia(invMass, radius, halfHeight);

    m_colliderStorage.shapes[colliderID.Index()] = ShapeReference(ShapeType::CAPSULE, ShapeID(capsuleIndex, m_colliderStorage.capsules.generations[capsuleIndex]));

    return colliderID;
}

void PhysicsWorld::DestroyCollider(ColliderID& colliderID)
{
    if (!IsValid(colliderID))
        return;

    const uint32_t index = colliderID.Index();
    const uint8_t generation = colliderID.Generation();

    switch (m_colliderStorage.shapes[index].type)
    {
    case ShapeType::BOX:
        DestroyBox(colliderID);
        break;
    case ShapeType::CAPSULE:
        DestroyCapsule(colliderID);
        break;
    case ShapeType::SPHERE:
        DestroySphere(colliderID);
        break;
    case ShapeType::NONE:
        break;
    }

    m_colliderStorage.shapes[index] = ShapeReference(ShapeType::NONE, ShapeID(ShapeID::INVALID));

    m_colliderStorage.availableIndices.push(index);
    m_colliderStorage.generations[index] = (generation + 1) & 0xFF;
    m_colliderStorage.alive[index] = false;

    const BodyID bodyID =m_colliderStorage.bodies[index];
    std::vector<ColliderID>& colliders = m_bodyStorage.colliders[bodyID.Index()];

    const auto it = std::ranges::find(colliders, colliderID);
    if (it != colliders.end())
    {
        *it = colliders.back();
        colliders.pop_back();
    }

    colliderID = ColliderID(ColliderID::INVALID);
}

void PhysicsWorld::AddLinearImpulse(const BodyID& bodyID, const Vec3& impulse)
{
    if (!IsValid(bodyID))
        return;

    const float invMass = m_bodyStorage.masses.inverseMasses[bodyID.Index()];
    if (invMass <= 0.0f)
        return;

    m_bodyStorage.velocities.linear[bodyID.Index()] += impulse * invMass;
}

void PhysicsWorld::AddAngularImpulse(const BodyID& bodyID, const Vec3& impulse)
{
    if (!IsValid(bodyID))
        return;

    if (m_bodyStorage.masses.inverseMasses[bodyID.Index()] <= 0.0f)
        return;

    const Mat3& invInertiaWorld = m_bodyStorage.masses.inverseInertiasWorld[bodyID.Index()];
    m_bodyStorage.velocities.angular[bodyID.Index()] += invInertiaWorld * impulse;
}

void PhysicsWorld::AddTorque(const BodyID& bodyID, const Vec3& torque)
{
    if (!IsValid(bodyID))
        return;

    if (m_bodyStorage.masses.inverseMasses[bodyID.Index()] <= 0.0f)
        return;

    m_bodyStorage.forces.torques[bodyID.Index()] += torque;
}

void PhysicsWorld::AddForce(const BodyID& bodyID, const Vec3& force)
{
    if (!IsValid(bodyID))
        return;

    if (m_bodyStorage.masses.inverseMasses[bodyID.Index()] <= 0.0f)
        return;

    m_bodyStorage.forces.forces[bodyID.Index()] += force;
}

void PhysicsWorld::UpdateWorldInertia()
{
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_bodyStorage.masses.inverseInertias.size()); ++i)
    {
        if (m_bodyStorage.masses.inverseMasses[i] <= 0.0f)
            continue;

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

ColliderID PhysicsWorld::AddCollider(const BodyID bodyID, const Vec3& positionOffset, const Quat& rotationOffset)
{
    uint32_t index;

    if (!m_colliderStorage.availableIndices.empty())
    {
        index = m_colliderStorage.availableIndices.front();
        m_colliderStorage.availableIndices.pop();
    }
    else
    {
        index = static_cast<uint32_t>(m_colliderStorage.localPositions.size());

        if (index >= ColliderID::MAX_BODY_ID)
            return ColliderID(ColliderID::INVALID);

        m_colliderStorage.bodies.emplace_back();

        m_colliderStorage.localPositions.emplace_back();
        m_colliderStorage.localRotations.emplace_back();

        m_colliderStorage.generations.push_back(0);
        m_colliderStorage.alive.emplace_back();

        m_colliderStorage.shapes.emplace_back();
    }

    m_colliderStorage.bodies[index] = bodyID;

    m_colliderStorage.localPositions[index] = positionOffset;
    m_colliderStorage.localRotations[index] = rotationOffset;

    m_colliderStorage.alive[index] = true;
    m_colliderStorage.shapes[index] = ShapeReference(ShapeType::NONE, ShapeID(ShapeID::INVALID));

    const ColliderID colliderID = ColliderID(index, m_colliderStorage.generations[index]);
    m_bodyStorage.colliders[bodyID.Index()].push_back(colliderID);

    return colliderID;
}

void PhysicsWorld::DestroySphere(const ColliderID& colliderID)
{
    const ShapeID shapeID = m_colliderStorage.shapes[colliderID.Index()].index;
    const uint32_t index = shapeID.Index();
    const uint8_t generation = shapeID.Generation();

    if (m_colliderStorage.spheres.generations[index] != generation)
        return;

    m_colliderStorage.spheres.availableIndices.push(index);
    m_colliderStorage.spheres.generations[index] = (generation + 1) & 0xFF;
}

void PhysicsWorld::DestroyBox(const ColliderID& colliderID)
{
    const ShapeID shapeID = m_colliderStorage.shapes[colliderID.Index()].index;
    const uint32_t index = shapeID.Index();
    const uint8_t generation = shapeID.Generation();

    if (m_colliderStorage.boxes.generations[index] != generation)
        return;

    m_colliderStorage.boxes.availableIndices.push(index);
    m_colliderStorage.boxes.generations[index] = (generation + 1) & 0xFF;
}

void PhysicsWorld::DestroyCapsule(const ColliderID& colliderID)
{
    const ShapeID shapeID = m_colliderStorage.shapes[colliderID.Index()].index;
    const uint32_t index = shapeID.Index();
    const uint8_t generation = shapeID.Generation();

    if (m_colliderStorage.capsules.generations[index] != generation)
        return;

    m_colliderStorage.capsules.availableIndices.push(index);
    m_colliderStorage.capsules.generations[index] = (generation + 1) & 0xFF;
}

bool PhysicsWorld::IsValid(const BodyID& bodyID) const
{
    if (!bodyID)
        return false;

    const uint32_t index = bodyID.Index();

    if (index >= static_cast<uint32_t>(m_bodyStorage.generations.size()))
        return false;

    return m_bodyStorage.generations[index] == bodyID.Generation() && m_bodyStorage.alive[index];
}

bool PhysicsWorld::IsValid(const ColliderID& colliderID) const
{
    if (!colliderID)
        return false;

    const uint32_t index = colliderID.Index();

    if (index >= static_cast<uint32_t>(m_colliderStorage.generations.size()))
        return false;

    return m_colliderStorage.generations[index] == colliderID.Generation() && m_colliderStorage.alive[index];
}

END_NAMESPACE_KAYOU
