/* GameController.h -- The engine/controller for the game.  Handles setup, looping, rendering, etc.  

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

#pragma once

#include "GlobalInclude.h"
#include "DebugDraw.h"

class GameController
{
public:
	GameController();
	~GameController();

	int OnExecute();

protected:
	/*window/SFML setup variables*/

	float width, height;
	//the main window, where drawing takes place
	sf::RenderWindow *window;

	/*Box2D setup variables*/

	//the main physics world
	b2World *world;
	//constants needed for stepping the physics simulation
	const float timeStep; //note: values for constants are set in the constructor's initialization list
	const int velocityIterations;
	const int positionIterations;

	/*game/demo variables*/

	//instruction/help message strings
	sf::String str1, str2, str3, str4;
	//the main Box2D body that moves with user input
	b2Body *body;
	//keep track of the small bodies for later use
	vector<b2Body*> smallBodyList;
	//basic shape rendering class for Box2D to use when "debug drawing"
	DebugDraw *debugDraw;

	void OnInit();
	void OnEvent(sf::Event &sfmlEvent);
	void OnLoop();
	void OnRender();
	void OnCleanup();
};
