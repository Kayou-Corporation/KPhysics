#include <iostream>

#include "KPhysics/Maths/AABB.hpp"
#include "KPhysics/Maths/Mat3.hpp"
#include "KPhysics/Maths/Mat3.hpp"
#include "KPhysics/Maths/Quaternion.hpp"
#include "KPhysics/Maths/Vec2.hpp"
#include "KPhysics/Maths/Vec3.hpp"
#include "KPhysics/Maths/Vec4.hpp"
#include "KPhysics/Maths/Scalar.hpp"
#include "KPhysics/KPhysics.hpp"

USING_KAYOU

int main()
{
    PhysicsWorld* world = new PhysicsWorld();
    BodyID id = world->CreateBody(Vec3(0.f), QuatIdentity(), 10.f);
    if (!id)
        return -1;

    ColliderID collID = world->AddSphereCollider(id, Vec3(0.f), QuatIdentity(), 1.f);
    if (!collID)
        return -1;

    world->AddLinearImpulse(id, Vec3(2.f, 20.f, 4.f));

    for (int i = 0; i < 100; ++i)
    {
        world->Step(1.f / 60.f);
        Vec3 pos = world->GetBodyPosition(id);
        std::cout << pos.x << " " << pos.y << " " << pos.z << '\n';
    }

    world->DestroyCollider(collID);
    world->DestroyBody(id); // DestroyBody also destroys all the body's attached colliders
    delete world;
    return 0;
}