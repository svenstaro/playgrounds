/* DebugDraw.cpp

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

#include "DebugDraw.hpp"

DebugDraw::DebugDraw(sf::RenderWindow *renderWindow)
{
	window = renderWindow;
}

const sf::Color DebugDraw::BtToSfColor(const btVector3& color) const {
	sf::Color result(sf::Uint8((color.x() * 255)), 
					 sf::Uint8((color.y() * 255)), 
					 sf::Uint8((color.z() * 255)));
	return result;
}

	void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
		sf::Shape line;
		line.AddPoint(from.x(), from.y(), BtToSfColor(color));
		line.AddPoint(to.x(), to.y(), BtToSfColor(color));

		window->Draw(line);
	}

	void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& from_color, const btVector3& to_color) {
		sf::Shape line;
		line.AddPoint(from.x(), from.y(), BtToSfColor(from_color));
		line.AddPoint(to.x(), to.y(), BtToSfColor(to_color));

		window->Draw(line);
	}

	/*
	void DebugDraw::drawSphere(btScalar radius, const btTransform& transform, const btVector3& color) {
		sf::Shape sphere = sf::Shape::Circle(transform.getOrigin.x, transform.getOrigin.y, radius, BtToSfColor(color));
	}

	void DebugDraw::drawSphere(const btVector3& p, btScalar radius, const btVector3& color) {
		sf::Shape sphere = sf::Shape::Circle(p.x, p.y, radius, BtToSfColor(color));
	}

	void DebugDraw::drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2, const btVector3&, const btVector3 &, const btVector3 &, const btVector3 &color, btScalar alpha) {
	}

	void DebugDraw::drawTriangle(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2, const btVector3 &color, btScalar);
	*/

	void DebugDraw::drawContactPoint(const btVector3& point_on_b, const btVector3& normal_on_b, btScalar distance, int life_time, const btVector3& color) {
		btVector3 to = point_on_b + normal_on_b * distance;
		const btVector3& from = point_on_b;

		drawLine(from, to, color);
	}

	void DebugDraw::reportErrorWarning(const char* warning_string) {
		std::cout << warning_string << std::endl;
	}

	void DebugDraw::draw3dText(const btVector3& location, const char* text_string) {
		sf::String string(text_string);
		sf::Text text(string);
		text.SetPosition(location.x(), location.y());
		
		window->Draw(text);
	}

	void DebugDraw::setDebugMode(int debugMode) {
		std::cout << "setDebugMode not implemented." << std::endl;
	}

	int DebugDraw::getDebugMode() const {
		std::cout << "getDebugMode not implemented." << std::endl;
	}

	/*
	void DebugDraw::drawAabb(const btVector3 &from, const btVector3 &to, const btVector3 &color);

	void DebugDraw::drawTransform(const btTransform &transform, btScalar orthoLen);

	void DebugDraw::drawArc(const btVector3 &center, const btVector3 &normal, const btVector3 &axis, btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle, const btVector3 &color, bool drawSect, btScalar stepDegrees=btScalar(10.f));

	void DebugDraw::drawSpherePatch(const btVector3 &center, const btVector3 &up, const btVector3 &axis, btScalar radius, btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3 &color, btScalar stepDegrees=btScalar(10.f));

	void DebugDraw::drawBox(const btVector3 &bbMin, const btVector3 &bbMax, const btVector3 &color);

	void DebugDraw::drawBox(const btVector3 &bbMin, const btVector3 &bbMax, const btTransform &trans, const btVector3 &color);
	*/
