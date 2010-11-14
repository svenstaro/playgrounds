#include "btBulletDynamicsCommon.h"
#include "DebugDraw.hpp"
#include "Definitions.hpp"
#include <SFML/Graphics.hpp>
#include <stdio.h>

int main(int argc, char** argv) {
	sf::RenderWindow* RenderWin = new sf::RenderWindow(sf::VideoMode(WIDTH, HEIGHT, 32), "lol test");
	RenderWin->UseVerticalSync(true);

	// Collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	// Use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded).
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	// The default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded).
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	// Set gravity to 9.8m/s² along y-axis.
	dynamicsWorld->setGravity(btVector3(0, 1, 0));

	// Get us some debug output. Without this, we'd see nothing at all.
	DebugDraw* debugDraw = new DebugDraw(RenderWin);
	debugDraw->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
	//debugDraw->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
	dynamicsWorld->setDebugDrawer(debugDraw);

	// Keep track of the shapes, we release memory at exit.
	// Make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	// Create a ground body.
	btScalar thickness(0.2);
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(WIDTH / 2 * METERS_PER_PIXEL), thickness, btScalar(0)));
	collisionShapes.push_back(groundShape);
	btTransform groundTransform(btQuaternion(0, 0, 0, 1), btVector3(WIDTH / 2 * METERS_PER_PIXEL, HEIGHT * METERS_PER_PIXEL, 0));
	// Using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects.
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo ground_rbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* ground_body = new btRigidBody(ground_rbInfo);
	// Add the body to the dynamics world.
	dynamicsWorld->addRigidBody(ground_body);

	// Create left wall.
	btTransform leftWallTransform(btQuaternion(0, 0, 1, 1), btVector3(0, HEIGHT / 2 * METERS_PER_PIXEL, 0));
	btDefaultMotionState* leftWallMotionState = new btDefaultMotionState(leftWallTransform);
	btRigidBody::btRigidBodyConstructionInfo leftWall_rbInfo(0, leftWallMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* leftwall_body = new btRigidBody(leftWall_rbInfo);
	// Add the body to the dynamics world.
	dynamicsWorld->addRigidBody(leftwall_body);

	// Create right wall.
	btTransform rightWallTransform(btQuaternion(0, 0, 1, 1), btVector3(WIDTH * METERS_PER_PIXEL, HEIGHT / 2 * METERS_PER_PIXEL, 0));
	btDefaultMotionState* rightWallMotionState = new btDefaultMotionState(rightWallTransform);
	btRigidBody::btRigidBodyConstructionInfo rightWall_rbInfo(0, rightWallMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* rightwall_body = new btRigidBody(rightWall_rbInfo);
	// Add the body to the dynamics world.
	dynamicsWorld->addRigidBody(rightwall_body);

	// Create ceiling
	btTransform topWallTransform(btQuaternion(0, 0, 0, 1), btVector3(WIDTH / 2 * METERS_PER_PIXEL, 0, 0));
	btDefaultMotionState* topWallMotionState = new btDefaultMotionState(topWallTransform);
	btRigidBody::btRigidBodyConstructionInfo topWall_rbInfo(0, topWallMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* topwall_body = new btRigidBody(topWall_rbInfo);
	// Add the body to the dynamics world.
	dynamicsWorld->addRigidBody(topwall_body);


	// Creare dynamic rigid body.

	//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
	btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass,localInertia);

	startTransform.setOrigin(btVector3(5, 5, 0));

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	dynamicsWorld->addRigidBody(body);

/// Do some simulation

	while(RenderWin->IsOpened()) {
		RenderWin->Clear(sf::Color(50,50,50));

		dynamicsWorld->stepSimulation(1.f/60.f, 10);
		
		//print positions of all objects
		for(int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--) {
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				btTransform trans;
				body->getMotionState()->getWorldTransform(trans);
				printf("world pos = %f,%f,%f\n", float(trans.getOrigin().getX()),float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
			}
		}

		//have Bullet use our debug drawing class to render the world
		dynamicsWorld->debugDrawWorld();

		RenderWin->Display();
	}


	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	for (int i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<collisionShapes.size();j++) {
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();

	return 0;
}
