/* DebugDraw.h -- An implementation of Box2D's b2DebugDraw interface for rendering simple shapes.  

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

#ifndef DEBUGDRAW_HPP
#define DEBUGDRAW_HPP

#include "GlobalInclude.hpp"

//a Box2D debug drawing class for simple rendering of physics objects
class DebugDraw : public btIDebugDraw
{
public:
	DebugDraw(sf::RenderWindow *renderWindow);

	// convert a Box2D (float 0.0f - 1.0f range) color to a SFML color (uint8 0 - 255 range)
	const sf::Color BtToSfColor(const btVector3& color) const;

	void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color);
	void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &fromColor, const btVector3 &toColor);
	/*void drawSphere(btScalar radius, const btTransform &transform, const btVector3 &color);
	void drawSphere(const btVector3 &p, btScalar radius, const btVector3 &color);
	void drawTriangle(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2, const btVector3 &, const btVector3 &, const btVector3 &, const btVector3 &color, btScalar alpha);
	void drawTriangle(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2, const btVector3 &color, btScalar);*/
	void drawContactPoint(const btVector3& point_on_b, const btVector3& normal_on_b, btScalar distance, int life_time, const btVector3& color);
	void reportErrorWarning(const char* warning_string);
	void draw3dText(const btVector3& location, const char* text_string);
	void setDebugMode(int debugMode);
	int getDebugMode() const;
	/*void drawAabb(const btVector3 &from, const btVector3 &to, const btVector3 &color);
	void drawTransform(const btTransform &transform, btScalar orthoLen);
	void drawArc(const btVector3 &center, const btVector3 &normal, const btVector3 &axis, btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle, const btVector3 &color, bool drawSect, btScalar stepDegrees=btScalar(10.f));
	void drawSpherePatch(const btVector3 &center, const btVector3 &up, const btVector3 &axis, btScalar radius, btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3 &color, btScalar stepDegrees=btScalar(10.f));
	void drawBox(const btVector3 &bbMin, const btVector3 &bbMax, const btVector3 &color);
	void drawBox(const btVector3 &bbMin, const btVector3 &bbMax, const btTransform &trans, const btVector3 &color);*/

private:
	sf::RenderWindow *window;
};

#endif
