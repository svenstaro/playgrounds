#include "DebugDraw.hpp"
#include "Definitions.hpp"

#include <SFML/Graphics.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btBox2dShape.h>
#include <BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h>
#include <BulletCollision/CollisionShapes/btConvex2dShape.h>
#include <BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_list.hpp>
#include <boost/foreach.hpp>

int main(int argc, char** argv) {
        sf::RenderWindow* RenderWin = new sf::RenderWindow(sf::VideoMode(WIDTH, HEIGHT, 32), "lol test");
        RenderWin->EnableVerticalSync(true);

        // Collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
		boost::shared_ptr<btDefaultCollisionConfiguration> collisionConfiguration(new btDefaultCollisionConfiguration());

        // Use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded).
		boost::shared_ptr<btCollisionDispatcher> dispatcher(new btCollisionDispatcher(collisionConfiguration.get()));

        // btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
		boost::shared_ptr<btBroadphaseInterface> broadphase(new btDbvtBroadphase());

        // The default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded).

		boost::shared_ptr<btVoronoiSimplexSolver> simplex(new btVoronoiSimplexSolver());
		boost::shared_ptr<btMinkowskiPenetrationDepthSolver> pd_solver(new btMinkowskiPenetrationDepthSolver());
		boost::shared_ptr<btSequentialImpulseConstraintSolver> solver(new btSequentialImpulseConstraintSolver());

		boost::shared_ptr<btDiscreteDynamicsWorld> dynamicsWorld(new btDiscreteDynamicsWorld(dispatcher.get(), broadphase.get(), solver.get(), collisionConfiguration.get()));

		boost::shared_ptr<btConvex2dConvex2dAlgorithm::CreateFunc> convex_algo_2d(new btConvex2dConvex2dAlgorithm::CreateFunc(simplex.get(),pd_solver.get()));
		
		dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE,CONVEX_2D_SHAPE_PROXYTYPE, convex_algo_2d.get());
		dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE,CONVEX_2D_SHAPE_PROXYTYPE, convex_algo_2d.get());
		dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE,BOX_2D_SHAPE_PROXYTYPE, convex_algo_2d.get());
		dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE,BOX_2D_SHAPE_PROXYTYPE, new btBox2dBox2dCollisionAlgorithm::CreateFunc());

        // Set gravity to 9.8m/s² along y-axis.
        dynamicsWorld->setGravity(btVector3(0, 1, 0));

        // Get us some debug output. Without this, we'd see nothing at all.
		boost::shared_ptr<DebugDraw> debugDraw(new DebugDraw(RenderWin));
        debugDraw->setDebugMode(btIDebugDraw::DBG_DrawWireframe);

        dynamicsWorld->setDebugDrawer(debugDraw.get());

        // Keep track of the shapes, we release memory at exit.
        // Make sure to re-use collision shapes among rigid bodies whenever possible!
        btAlignedObjectArray<btCollisionShape*> collisionShapes;

        // Create a ground body.
        btScalar thickness(0.2);
		boost::shared_ptr<btCollisionShape> groundShape(new btBoxShape(btVector3(btScalar(WIDTH / 2 * METERS_PER_PIXEL), thickness, btScalar(10))));
        collisionShapes.push_back(groundShape.get());
        btTransform groundTransform(btQuaternion(0, 0, 0, 1), btVector3(WIDTH / 2 * METERS_PER_PIXEL, HEIGHT * METERS_PER_PIXEL, 0));
        // Using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects.
		boost::shared_ptr<btDefaultMotionState> groundMotionState(new btDefaultMotionState(groundTransform));
        btRigidBody::btRigidBodyConstructionInfo ground_rbInfo(0, groundMotionState.get(), groundShape.get(), btVector3(0, 0, 0));
		boost::shared_ptr<btRigidBody> ground_body(new btRigidBody(ground_rbInfo));
		ground_body->setLinearFactor(btVector3(1,1,0));
		ground_body->setAngularFactor(btVector3(0,0,1));
        // Add the body to the dynamics world.
        dynamicsWorld->addRigidBody(ground_body.get());

        // Create left wall.
        btTransform leftWallTransform(btQuaternion(0, 0, 1, 1), btVector3(0, HEIGHT / 2 * METERS_PER_PIXEL, 0));
		boost::shared_ptr<btDefaultMotionState> leftWallMotionState(new btDefaultMotionState(leftWallTransform));
        btRigidBody::btRigidBodyConstructionInfo leftWall_rbInfo(0, leftWallMotionState.get(), groundShape.get(), btVector3(0, 0, 0));
		boost::shared_ptr<btRigidBody> leftwall_body(new btRigidBody(leftWall_rbInfo));
		leftwall_body->setLinearFactor(btVector3(1,1,0));
		leftwall_body->setAngularFactor(btVector3(0,0,1));
        // Add the body to the dynamics world.
        dynamicsWorld->addRigidBody(leftwall_body.get());

        // Create right wall.
        btTransform rightWallTransform(btQuaternion(0, 0, 1, 1), btVector3(WIDTH * METERS_PER_PIXEL, HEIGHT / 2 * METERS_PER_PIXEL, 0));
		boost::shared_ptr<btDefaultMotionState> rightWallMotionState(new btDefaultMotionState(rightWallTransform));
        btRigidBody::btRigidBodyConstructionInfo rightWall_rbInfo(0, rightWallMotionState.get(), groundShape.get(), btVector3(0, 0, 0));
		boost::shared_ptr<btRigidBody> rightwall_body(new btRigidBody(rightWall_rbInfo));
		rightwall_body->setLinearFactor(btVector3(1,1,0));
		rightwall_body->setAngularFactor(btVector3(0,0,1));
        // Add the body to the dynamics world.
        dynamicsWorld->addRigidBody(rightwall_body.get());

        // Create ceiling
        btTransform topWallTransform(btQuaternion(0, 0, 0, 1), btVector3(WIDTH / 2 * METERS_PER_PIXEL, 0, 0));
		boost::shared_ptr<btDefaultMotionState> topWallMotionState(new btDefaultMotionState(topWallTransform));
        btRigidBody::btRigidBodyConstructionInfo topWall_rbInfo(0, topWallMotionState.get(), groundShape.get(), btVector3(0, 0, 0));
		boost::shared_ptr<btRigidBody> topwall_body(new btRigidBody(topWall_rbInfo));
		topwall_body->setLinearFactor(btVector3(1,1,0));
		topwall_body->setAngularFactor(btVector3(0,0,1));
        // Add the body to the dynamics world.
        dynamicsWorld->addRigidBody(topwall_body.get());


        // Create dynamic rigid body.

        //btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		boost::shared_ptr<btCollisionShape> colShape(new btSphereShape(btScalar(0.6)));
        collisionShapes.push_back(colShape.get());

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar mass(1.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
                colShape->calculateLocalInertia(mass,localInertia);

        startTransform.setOrigin(btVector3(2, 5, 0));

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		boost::shared_ptr<btDefaultMotionState> myMotionState(new btDefaultMotionState(startTransform));
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState.get(),colShape.get(),localInertia);
		boost::shared_ptr<btRigidBody> body(new btRigidBody(rbInfo));
		body->setLinearFactor(btVector3(1,1,0));
		body->setAngularFactor(btVector3(0,0,1));

        dynamicsWorld->addRigidBody(body.get());

		// Create lulz
		boost::ptr_list<btRigidBody> body_list;
		boost::ptr_list<btDefaultMotionState> motionstate_list;
		boost::ptr_list<btCollisionShape> colshape_list;
		for (int i=0;i <= 10; ++i) {
			if (i < 5)
				colshape_list.push_back(new btSphereShape(btScalar(sf::Randomizer::Random(0.1f, 0.8f))));
			else
				colshape_list.push_back(new btBoxShape(btVector3(sf::Randomizer::Random(0.1f,0.8f), sf::Randomizer::Random(0.1f,0.8f), 10)));
			if (isDynamic)
                colshape_list.back().calculateLocalInertia(mass,localInertia);
			collisionShapes.push_back(&(colshape_list.back()));
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(i,i,0));
			motionstate_list.push_back(new btDefaultMotionState(startTransform));
			btRigidBody* lol = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass,&(motionstate_list.back()),&(colshape_list.back()),localInertia));
			lol->setLinearFactor(btVector3(1,1,0));
			lol->setAngularFactor(btVector3(0,0,1));
            body_list.push_back(lol);
		}
		BOOST_FOREACH (btRigidBody& body, body_list) {
			dynamicsWorld->addRigidBody(&body);
		}

/// Do some simulation
		sf::Event sfmlEvent;
		sf::Clock mClock;
		sf::Text mText;

        while(RenderWin->IsOpened()) {
			while(RenderWin->GetEvent(sfmlEvent)) {
				if(sfmlEvent.Type == sf::Event::Closed) 
					RenderWin->Close();
				if(sfmlEvent.Type == sf::Event::KeyPressed) {
					if(sfmlEvent.Key.Code == sf::Key::Escape) 
						RenderWin->Close();
				}
			}

			float time_delta = mClock.GetElapsedTime();
			mClock.Reset();


			// SFML access class for real-time input
			const sf::Input& input = RenderWin->GetInput();
			// get the current "frame time" (seconds elapsed since the previous frame, hopefully close to 1/60 since vsync is enabled)
			float frameTime = RenderWin->GetFrameTime();

			//step the Box2D physics world, with a constant time step
			//note that this is kind of a bad way to do time steps, as it can get out of sync if the framerate drops (see http://gafferongames.wordpress.com/game-physics/fix-your-timestep/ for more information)
			dynamicsWorld->stepSimulation(1/60.f, 10);
			dynamicsWorld->clearForces();

			//check for user keyboard input to control Bullet forces/torques/etc
			float mag = 1000.0f;
			if(input.IsKeyDown(sf::Key::Left))
				body->applyForce(btVector3(-mag*frameTime, 0.0f, 0), body->getCenterOfMassPosition());
			if(input.IsKeyDown(sf::Key::Right)) 
				body->applyForce(btVector3(mag*frameTime, 0.0f, 0), body->getCenterOfMassPosition());
			if(input.IsKeyDown(sf::Key::Up)) 
				body->applyForce(btVector3(0.0f, -mag*frameTime, 0), body->getCenterOfMassPosition());
			if(input.IsKeyDown(sf::Key::Down)) 
				body->applyForce(btVector3(0.0f, mag*frameTime, 0), body->getCenterOfMassPosition());
			if(input.IsKeyDown(sf::Key::Q)) 
				body->applyTorque(btVector3(mag/2*frameTime, 0, 0));
			if(input.IsKeyDown(sf::Key::E)) 
				body->applyTorque(btVector3(-mag/2*frameTime, 0, 0));

			// apply random forces to all the small bodies created earlier
			if(input.IsKeyDown(sf::Key::Space)) {
                BOOST_FOREACH (btRigidBody& body, body_list) {
				    body.applyForce(btVector3(mag/100*(10-rand()%20), mag/100*(10-rand()%20), 0), body.getCenterOfMassPosition());
                }
			}

			if(input.IsKeyDown(sf::Key::W)) {
				// make a new gravity vector
				btVector3 gravity(0.0f, -2.0f, 0);
				// pass it to Box2D
				dynamicsWorld->setGravity(gravity);
				// wake up all sleeping bodies so they will be affected by the gravity change
				//BOOST_FOREACH(btRigidBody* body, smallBodyList)
				//	body->SetAwake(true);
			}
			if(input.IsKeyDown(sf::Key::S)) {
				btVector3 gravity(0.0f, 2.0f, 0);
				dynamicsWorld->setGravity(gravity);
				//BOOST_FOREACH(btRigidBody* body, smallBodyList)
				//	body->SetAwake(true);
			}
			if(input.IsKeyDown(sf::Key::A)) {
				btVector3 gravity(-2.0f, 0.0f, 0);
				dynamicsWorld->setGravity(gravity);
				//BOOST_FOREACH(btRigidBody* body, smallBodyList)
				//	body->SetAwake(true);
			}
			if(input.IsKeyDown(sf::Key::D)) {
				btVector3 gravity(2.0f, 0.0f, 0);
				dynamicsWorld->setGravity(gravity);
				//BOOST_FOREACH(btRigidBody* body, smallBodyList)
				//	body->SetAwake(true);
			}
                RenderWin->Clear(sf::Color(50,50,50));


                //print positions of all objects
                for(int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--) {
                        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
                        btRigidBody* tmp_body = btRigidBody::upcast(obj);
                        if (tmp_body && tmp_body->getMotionState() && tmp_body != body.get()) {
                                btVector3 tmp_body_pos = tmp_body->getCenterOfMassPosition();
                                btVector3 body_pos = body->getCenterOfMassPosition();
                                //std::cout << body_pos.distance(tmp_body_pos) << std::endl;
                                if(body_pos.distance(tmp_body_pos) < 5.f) {
                                    //btVector3 force(0, -10, 0);
                                    btVector3 force(body_pos - tmp_body_pos);
                                    //force = -force;
                                    tmp_body->applyCentralForce(force);
                                    std::cout << "lol" << std::endl;
                                }
                                //std::cout << body_pos.getX() << std::endl;
                                //btTransform tmp_body_trans;
                                //tmp_body->getMotionState()->getWorldTransform(tmp_body_trans);

                                //btTransform body_trans;
                                //body->getMotionState()->getWorldTransform(body_trans);
                                //printf("world pos = %f,%f,%f\n", float(trans.getOrigin().getX()),float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
                        }
                }

                //have Bullet use our debug drawing class to render the world
                dynamicsWorld->debugDrawWorld();

				mText = sf::Text(boost::lexical_cast<std::string>(1.f/time_delta));
				mText.SetPosition(5,5);
				RenderWin->Draw(mText);

                RenderWin->Display();
        }

        return 0;
}
