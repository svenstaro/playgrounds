/* GameController.cpp

  Copyright (C) 2010 Allen Jordan

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Allen Jordan allen.jordan@gmail.com
*/

#include "GameController.h"

GameController::GameController() : 
	timeStep(1.0f/60.0f), 
	velocityIterations(10), 
	positionIterations(10)
{
	
}

GameController::~GameController()
{
}

//public method for starting and running the game
int GameController::OnExecute()
{
	OnInit();

	sf::Event sfmlEvent;
	//start the game loop
	while (window->IsOpened())
	{
		//check for window events
		while (window->GetEvent(sfmlEvent))
		{
			OnEvent(sfmlEvent);
		}

		OnLoop();
		OnRender();
	}

	OnCleanup();

	return EXIT_SUCCESS;
}

//initialize the game
void GameController::OnInit()
{
	//create a new SFML rendering window
	window = new sf::RenderWindow(sf::VideoMode(800, 600, 32), "SFML Test");
	//prevent crazy frame rates (and make the framerate pseudo-sync with the physics stepping, though this is a crappy method of doing so)
	window->UseVerticalSync(true);
	width = (float)window->GetWidth();
	height = (float)window->GetHeight();

	//define the Box2D gravity vector
	b2Vec2 gravity(0.0f, 2.0f);
	world = new b2World(gravity, true);

	//create our debug drawing object and pass it to Box2D
	debugDraw = new DebugDraw(window);
	debugDraw->SetFlags(b2DebugDraw::e_shapeBit);
	world->SetDebugDraw(debugDraw);

	//define the static ground body, keeping the shapes from falling into oblivion
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(width/2*METERS_PER_PIXEL, height*METERS_PER_PIXEL);
	//create ground body (static body)
	b2Body *groundBody = world->CreateBody(&groundBodyDef);
	//create a polygon shape for the ground
	b2PolygonShape groundBox;
	groundBox.SetAsBox(width/2*METERS_PER_PIXEL, 5*METERS_PER_PIXEL);
	//fix the ground box shape to the ground body
	groundBody->CreateFixture(&groundBox, 0.0f);

	//create the left side/wall
	b2BodyDef leftSideBodyDef;
	leftSideBodyDef.position.Set(0.0f, height/2*MPP);
	b2Body *leftSideBody = world->CreateBody(&leftSideBodyDef);
	b2PolygonShape leftSideBox;
	leftSideBox.SetAsBox(5*MPP, height/2*MPP);
	leftSideBody->CreateFixture(&leftSideBox, 0.0f);

	//create the right side/wall
	b2BodyDef rightSideBodyDef;
	rightSideBodyDef.position.Set(width*MPP, height/2*MPP);
	b2Body *rightSideBody = world->CreateBody(&rightSideBodyDef);
	b2PolygonShape rightSideBox;
	rightSideBox.SetAsBox(5*MPP, height/2*MPP);
	rightSideBody->CreateFixture(&rightSideBox, 0.0f);

	//create the top ceiling
	b2BodyDef topBodyDef;
	topBodyDef.position.Set(width/2*MPP, 0.0f);
	b2Body *topBody = world->CreateBody(&topBodyDef);
	b2PolygonShape topBox;
	topBox.SetAsBox(width/2*MPP, 5*MPP);
	topBody->CreateFixture(&topBox, 0.0f);

	//define a dynamic body square that will move around with user input
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(width/2*METERS_PER_PIXEL, height/2*METERS_PER_PIXEL);
	//create the dynamic body
	body = world->CreateBody(&bodyDef);
	//shape of the dynamic body
	b2PolygonShape dynamicBox;
	dynamicBox.SetAsBox(64.0f/2*METERS_PER_PIXEL, 64.0f/2*METERS_PER_PIXEL);
	//fix the shape to the dynamic body
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.5f;
	//apply the fixture to the body
	body->CreateFixture(&fixtureDef);

	//define a stack of square bodies
	for (int i=1; i<17; i++)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(200*MPP, (10+i*32)*MPP);
		b2Body *b = world->CreateBody(&bd);
		b2PolygonShape box;
		box.SetAsBox(16*MPP, 16*MPP);
		b2FixtureDef fd;
		fd.shape = &box;
		fd.density = 0.5f;
		fd.friction = 0.3f;
		fd.restitution = 0.5f;
		b->CreateFixture(&fd);

		smallBodyList.push_back(b);
	}

	//define a 'U' shaped area of square boxes
	for (int i=1; i<=15; i++)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;

		if (i <= 5)
			bd.position.Set(500*MPP, (400+i*32)*MPP);
		else if (i <= 10)
			bd.position.Set((505 + (i-5)*32)*MPP, 500*MPP);
		else
			bd.position.Set(702*MPP, (400+(i-10)*32)*MPP);
		b2Body *b = world->CreateBody(&bd);
		b2PolygonShape box;
		box.SetAsBox(16*MPP, 16*MPP);
		b2FixtureDef fd;
		fd.shape = &box;
		fd.density = 1.0f;
		fd.friction = 0.3f;
		fd.restitution = 0.5f;
		b->CreateFixture(&fd);

		smallBodyList.push_back(b);
	}

	//make some circles to put in the 'U' shape we made earlier
	for (int i=0; i<10; i++)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(600*MPP, (32 + (i*32))*MPP);
		b2Body *b = world->CreateBody(&bd);
		b2CircleShape circle;
		//circle.m_p.Set(50*MPP, 50*MPP);
		circle.m_radius = (2+rand()%15)*MPP;
		b2FixtureDef fd;
		fd.shape = &circle;
		fd.density = 0.5f;
		fd.friction = 0.3f;
		fd.restitution = 0.5f;
		b->CreateFixture(&fd);

		smallBodyList.push_back(b);
	}

	//control instructions as SFML strings to draw onto the window
	str1.SetText("arrows = move box");
	str1.SetSize(18.0f);
	str1.SetPosition(10, 10);
	str2.SetText("[ ] = rotate box");
	str2.SetSize(18.0f);
	str2.SetPosition(10, 30);
	str3.SetText("space = random crazy forces");
	str3.SetSize(18.0f);
	str3.SetPosition(10, 50);
	str4.SetText("w s a d = change gravity");
	str4.SetSize(18.0f);
	str4.SetPosition(10, 70);
}

//process events
void GameController::OnEvent(sf::Event &sfmlEvent)
{
	if (sfmlEvent.Type == sf::Event::Closed) window->Close();
	if (sfmlEvent.Type == sf::Event::KeyPressed)
	{
		if (sfmlEvent.Key.Code == sf::Key::Escape) window->Close();
	}
}

//perform game logic
void GameController::OnLoop()
{
	//SFML access class for real-time input
	const sf::Input &input = window->GetInput();

	//get the current "frame time" (seconds elapsed since the previous frame, hopefully close to 1/60 since vsync is enabled)
	float frameTime = window->GetFrameTime();

	//step the Box2D physics world, with a constant time step
	//note that this is kind of a bad way to do time steps, as it can get out of sync if the framerate drops (see http://gafferongames.wordpress.com/game-physics/fix-your-timestep/ for more information)
	world->Step(timeStep, velocityIterations, positionIterations);
	world->ClearForces();

	//check for user keyboard input to control Box2D forces/torques/etc
	float mag = 500.0f;
	if (input.IsKeyDown(sf::Key::Left)) body->ApplyForce(b2Vec2(-mag*frameTime, 0.0f), body->GetPosition());
	if (input.IsKeyDown(sf::Key::Right)) body->ApplyForce(b2Vec2(mag*frameTime, 0.0f), body->GetPosition());
	if (input.IsKeyDown(sf::Key::Up)) body->ApplyForce(b2Vec2(0.0f, -mag*frameTime), body->GetPosition());
	if (input.IsKeyDown(sf::Key::Down)) body->ApplyForce(b2Vec2(0.0f, mag*frameTime), body->GetPosition());
	if (input.IsKeyDown(sf::Key::RBracket)) body->ApplyTorque(mag/2*frameTime);
	if (input.IsKeyDown(sf::Key::LBracket)) body->ApplyTorque(-mag/2*frameTime);
	//apply random forces to all the small bodies created earlier
	if (input.IsKeyDown(sf::Key::Space))
	{
		for (sf::Uint32 i=0; i<smallBodyList.size(); i++)
		{
			smallBodyList[i]->ApplyForce(b2Vec2(mag/100*(10-rand()%20), mag/100*(10-rand()%20)), smallBodyList[i]->GetPosition());
		}
	}
	if (input.IsKeyDown(sf::Key::W))
	{
		//make a new gravity vector
		b2Vec2 gravity(0.0f, -2.0f);
		//pass it to Box2D
		world->SetGravity(gravity);
		//wake up all sleeping bodies so they will be affected by the gravity change
		BOOST_FOREACH(b2Body* body, smallBodyList)
		{
			body->SetAwake(true);
		}
	}
	if (input.IsKeyDown(sf::Key::S))
	{
		b2Vec2 gravity(0.0f, 2.0f);
		world->SetGravity(gravity);
		BOOST_FOREACH(b2Body* body, smallBodyList)
		{
			body->SetAwake(true);
		}
	}
	if (input.IsKeyDown(sf::Key::A))
	{
		b2Vec2 gravity(-2.0f, 0.0f);
		world->SetGravity(gravity);
		BOOST_FOREACH(b2Body* body, smallBodyList)
		{
			body->SetAwake(true);
		}
	}
	if (input.IsKeyDown(sf::Key::D))
	{
		b2Vec2 gravity(2.0f, 0.0f);
		world->SetGravity(gravity);
		BOOST_FOREACH(b2Body* body, smallBodyList)
		{
			body->SetAwake(true);
		}
	}
}

//render the scene
void GameController::OnRender()
{
	//clear the window's rendering area before drawing
	window->Clear();

	//have Box2D use our debug drawing class to render the world
	world->DrawDebugData();

	//draw the control help strings
	window->Draw(str1);
	window->Draw(str2);
	window->Draw(str3);
	window->Draw(str4);

	//update the rendering window with all the latest drawn items
	window->Display();
}

//cleanup before exiting
void GameController::OnCleanup()
{
	delete window;
	delete world;
	delete debugDraw;
}
