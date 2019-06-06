/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package box2D.collision;


import box2D.collision.shapes.B2CircleShape;
import box2D.collision.shapes.B2PolygonShape;
import box2D.collision.shapes.B2Shape;
import box2D.collision.shapes.B2ShapeType;
import box2D.common.B2Settings;
import box2D.common.math.B2Vec2;


/**
 * A distance proxy is used by the GJK algorithm.
 * It encapsulates any shape.
 */
class B2DistanceProxy 
{
	/**
	 * Initialize the proxy using the given shape. The shape
	 * must remain in scope while the proxy is in use.
	 */
	public function set(shape:B2Shape):Void
	{
		switch(shape.getType())
		{
			case B2ShapeType.CIRCLE_SHAPE:
			{
				var circle:B2CircleShape = cast (shape, B2CircleShape);
				m_isCircle = true;
				m_circleVertex = circle.m_p;
				m_polygonVertices = null;
				m_count = 1;
				m_radius = circle.m_radius;
			}
			
			case B2ShapeType.POLYGON_SHAPE:
			{
				var polygon:B2PolygonShape =  cast (shape, B2PolygonShape);
				m_isCircle = false;
				m_circleVertex = null;
				m_polygonVertices = polygon.m_vertices;
				m_count = polygon.m_vertexCount;
				m_radius = polygon.m_radius;
			}
			
			default:
			B2Settings.b2Assert(false);
		}
	}
	
	/**
	 * Get the supporting vertex index in the given direction.
	 */
	public function getSupport(d:B2Vec2):Float
	{
		if (m_isCircle)
		{
			return 0;
		}
		else
		{
			var bestIndex:Int = 0;
			var bestValue:Float = m_polygonVertices[0].x * d.x + m_polygonVertices[0].y * d.y;
			for (i in 1...m_count)
			{
				var value:Float = m_polygonVertices[i].x * d.x + m_polygonVertices[i].y * d.y;
				if (value > bestValue)
				{
					bestIndex = i;
					bestValue = value;
				}
			}
			return bestIndex;
		}
	}
	
	/**
	 * Get the supporting vertex in the given direction.
	 */
	public function getSupportVertex(d:B2Vec2):B2Vec2
	{
		if (m_isCircle)
		{
			return m_circleVertex;
		}
		else
		{
			var bestIndex:Int = 0;
			var bestValue:Float = m_polygonVertices[0].x * d.x + m_polygonVertices[0].y * d.y;
			for (i in 1...m_count)
			{
				var value:Float = m_polygonVertices[i].x * d.x + m_polygonVertices[i].y * d.y;
				if (value > bestValue)
				{
					bestIndex = i;
					bestValue = value;
				}
			}
			return m_polygonVertices[bestIndex];
		}
	}
	
	/**
	 * Get the vertex count.
	 */
	public function getVertexCount():Int
	{
		return m_count;
	}
	
	/**
	 * Get a vertex by index. Used by b2Distance.
	 */
	public function getVertex(index:Int):B2Vec2
	{
		B2Settings.b2Assert(0 <= index && index < m_count);
		if (m_isCircle)
		{
			return m_circleVertex;
		}
		else
		{
			return m_polygonVertices[index];
		}
	}
	
	
	public function new () {
		
		m_isCircle = false;
		m_circleVertex = null;
		m_polygonVertices = null;
		
	}
	
	public var m_isCircle:Bool;
	public var m_circleVertex:B2Vec2;
	public var m_polygonVertices:Array <B2Vec2>;
	public var m_count:Int = 0;
	public var m_radius:Float = 0;
}