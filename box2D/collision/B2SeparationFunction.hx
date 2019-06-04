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


import box2D.common.B2Settings;
import box2D.common.math.B2Mat22;
import box2D.common.math.B2Math;
import box2D.common.math.B2Transform;
import box2D.common.math.B2Vec2;


class B2SeparationFunction
{
	//enum Type
	//public static var e_points:Int = 0x01;
	//public static var e_faceA:Int = 0x02;
	//public static var e_faceB:Int = 0x04;
	
	public function initialize(cache:B2SimplexCache,
								proxyA:B2DistanceProxy, transformA:B2Transform,
								proxyB:B2DistanceProxy, transformB:B2Transform):Void
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		var count:Int = cache.count;
		B2Settings.b2Assert(0 < count && count < 3);
		
		var localPointA:B2Vec2 = new B2Vec2 ();
		var localPointA1:B2Vec2;
		var localPointA2:B2Vec2;
		var localPointB:B2Vec2 = new B2Vec2 ();
		var localPointB1:B2Vec2;
		var localPointB2:B2Vec2;
		var pointAX:Float;
		var pointAY:Float;
		var pointBX:Float;
		var pointBY:Float;
		var normalX:Float;
		var normalY:Float;
		var tMat:B2Mat22;
		var tVec:B2Vec2;
		var s:Float;
		var sgn:Float;
		
		if (count == 1)
		{
			m_type = B2SeparationFunctionType.POINTS;
			localPointA = m_proxyA.getVertex(cache.indexA[0]);
			localPointB = m_proxyB.getVertex(cache.indexB[0]);
			//pointA = b2Math.b2MulX(transformA, localPointA);
			tVec = localPointA;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//pointB = b2Math.b2MulX(transformB, localPointB);
			tVec = localPointB;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//m_axis = b2Math.SubtractVV(pointB, pointA);
			m_axis.x = pointBX - pointAX;
			m_axis.y = pointBY - pointAY;
			m_axis.normalize();
		}
		else if (cache.indexB[0] == cache.indexB[1])
		{
			// Two points on A and one on B
			m_type = B2SeparationFunctionType.FACE_A;
			localPointA1 = m_proxyA.getVertex(cache.indexA[0]);
			localPointA2 = m_proxyA.getVertex(cache.indexA[1]);
			localPointB = m_proxyB.getVertex(cache.indexB[0]);
			m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
			m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
			B2Math.crossVF(B2Math.subtractVV(localPointA2, localPointA1, m_axis), 1.0, m_axis);
			m_axis.normalize();
			
			//normal = b2Math.b2MulMV(transformA.R, m_axis);
			tVec = m_axis;
			tMat = transformA.R;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//pointA = b2Math.b2MulX(transformA, m_localPoint);
			tVec = m_localPoint;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//pointB = b2Math.b2MulX(transformB, localPointB);
			tVec = localPointB;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			
			//float32 s = b2Dot(pointB - pointA, normal);
			s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
			if (s < 0.0)
			{
				m_axis.negativeSelf();
			}
		}
		else if (cache.indexA[0] == cache.indexA[0])
		{
			// Two points on B and one on A
			m_type = B2SeparationFunctionType.FACE_B;
			localPointB1 = m_proxyB.getVertex(cache.indexB[0]);
			localPointB2 = m_proxyB.getVertex(cache.indexB[1]);
			localPointA = m_proxyA.getVertex(cache.indexA[0]);
			m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
			m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
			B2Math.crossVF(B2Math.subtractVV(localPointB2, localPointB1, m_axis), 1.0, m_axis);
			m_axis.normalize();
			
			//normal = b2Math.b2MulMV(transformB.R, m_axis);
			tVec = m_axis;
			tMat = transformB.R;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//pointB = b2Math.b2MulX(transformB, m_localPoint);
			tVec = m_localPoint;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//pointA = b2Math.b2MulX(transformA, localPointA);
			tVec = localPointA;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			
			//float32 s = b2Dot(pointA - pointB, normal);
			s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
			if (s < 0.0)
			{
				m_axis.negativeSelf();
			}
		}
		else
		{
			// Two points on B and two points on A.
			// The faces are parallel.
			localPointA1 = m_proxyA.getVertex(cache.indexA[0]);
			localPointA2 = m_proxyA.getVertex(cache.indexA[1]);
			localPointB1 = m_proxyB.getVertex(cache.indexB[0]);
			localPointB2 = m_proxyB.getVertex(cache.indexB[1]);
			
			var pA:B2Vec2 = B2Math.mulX(transformA, localPointA, new B2Vec2());
			var dA:B2Vec2 = B2Math.mulMV(transformA.R, B2Math.subtractVV(localPointA2, localPointA1, new B2Vec2()), new B2Vec2());
			var pB:B2Vec2 = B2Math.mulX(transformB, localPointB, new B2Vec2());
			var dB:B2Vec2 = B2Math.mulMV(transformB.R, B2Math.subtractVV(localPointB2, localPointB1, new B2Vec2()), new B2Vec2());
			
			var a:Float = dA.x * dA.x + dA.y * dA.y;
			var e:Float = dB.x * dB.x + dB.y * dB.y;
			var r:B2Vec2 = B2Math.subtractVV(dB, dA, new B2Vec2());
			var c:Float = dA.x * r.x + dA.y * r.y;
			var f:Float = dB.x * r.x + dB.y * r.y;
			
			var b:Float = dA.x * dB.x + dA.y * dB.y;
			var denom:Float = a * e-b * b;
			
			s = 0.0;
			if (denom != 0.0)
			{
				s = B2Math.clamp((b * f - c * e) / denom, 0.0, 1.0);
			}
			
			var t:Float = (b * s + f) / e;
			if (t < 0.0)
			{
				t = 0.0;
				s = B2Math.clamp((b - c) / a, 0.0, 1.0);
			}
			
			//b2Vec2 localPointA = localPointA1 + s * (localPointA2 - localPointA1);
			localPointA = new B2Vec2();
			localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
			localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
			//b2Vec2 localPointB = localPointB1 + s * (localPointB2 - localPointB1);
			localPointB = new B2Vec2();
			localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
			localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
			
			if (s == 0.0 || s == 1.0)
			{
				m_type = B2SeparationFunctionType.FACE_B;
				B2Math.crossVF(B2Math.subtractVV(localPointB2, localPointB1, m_axis), 1.0, m_axis);
				m_axis.normalize();
                
				m_localPoint = localPointB;
				
				//normal = b2Math.b2MulMV(transformB.R, m_axis);
				tVec = m_axis;
				tMat = transformB.R;
				normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				//pointB = b2Math.b2MulX(transformB, m_localPoint);
				tVec = m_localPoint;
				tMat = transformB.R;
				pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				//pointA = b2Math.b2MulX(transformA, localPointA);
				tVec = localPointA;
				tMat = transformA.R;
				pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				//float32 sgn = b2Dot(pointA - pointB, normal);
				sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
				if (s < 0.0)
				{
					m_axis.negativeSelf();
				}
			}
			else
			{
				m_type = B2SeparationFunctionType.FACE_A;
				B2Math.crossVF(B2Math.subtractVV(localPointA2, localPointA1, m_axis), 1.0, m_axis);
				
				m_localPoint = localPointA;
				
				//normal = b2Math.b2MulMV(transformA.R, m_axis);
				tVec = m_axis;
				tMat = transformA.R;
				normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				//pointA = b2Math.b2MulX(transformA, m_localPoint);
				tVec = m_localPoint;
				tMat = transformA.R;
				pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				//pointB = b2Math.b2MulX(transformB, localPointB);
				tVec = localPointB;
				tMat = transformB.R;
				pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				//float32 sgn = b2Dot(pointB - pointA, normal);
				sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
				if (s < 0.0)
				{
					m_axis.negativeSelf();
				}
			}
		}
	}

	private static var s_axisA:B2Vec2 = new B2Vec2();
	private static var s_axisB:B2Vec2 = new B2Vec2();
	private static var s_pointA:B2Vec2 = new B2Vec2();
	private static var s_pointB:B2Vec2 = new B2Vec2();
	private static var s_normal:B2Vec2 = new B2Vec2();
	public function evaluate(transformA:B2Transform, transformB:B2Transform):Float
	{
		var localPointA:B2Vec2;
		var localPointB:B2Vec2;
		var seperation:Float;
		switch(m_type)
		{
			case POINTS:
			{
				B2Math.mulTMV(transformA.R, m_axis, s_axisA);
				B2Math.mulTMV(transformB.R, m_axis.getNegative(s_axisB), s_axisB);
				localPointA = m_proxyA.getSupportVertex(s_axisA);
				localPointB = m_proxyB.getSupportVertex(s_axisB);
				B2Math.mulX(transformA, localPointA, s_pointA);
				B2Math.mulX(transformB, localPointB, s_pointB);
				//float32 separation = b2Dot(pointB - pointA, m_axis);
				seperation = (s_pointB.x - s_pointA.x) * m_axis.x + (s_pointB.y - s_pointA.y) * m_axis.y;
				return seperation;
			}
			case FACE_A:
			{
				B2Math.mulMV(transformA.R, m_axis, s_normal);
				B2Math.mulX(transformA, m_localPoint, s_pointA);
				
				B2Math.mulTMV(transformB.R, s_normal.getNegative(s_axisB), s_axisB);
				
				localPointB = m_proxyB.getSupportVertex(s_axisB);
				B2Math.mulX(transformB, localPointB, s_pointB);
				
				//float32 separation = b2Dot(pointB - pointA, normal);
				seperation = (s_pointB.x - s_pointA.x) * s_normal.x + (s_pointB.y - s_pointA.y) * s_normal.y;
				return seperation;
			}
			case FACE_B:
			{
				B2Math.mulMV(transformB.R, m_axis, s_normal);
				B2Math.mulX(transformB, m_localPoint, s_pointB);
				
				B2Math.mulTMV(transformA.R, s_normal.getNegative(s_axisA), s_axisA);
				
				localPointA = m_proxyA.getSupportVertex(s_axisA);
				B2Math.mulX(transformA, localPointA, s_pointA);
				
				//float32 separation = b2Dot(pointA - pointB, normal);
				seperation = (s_pointA.x - s_pointB.x) * s_normal.x + (s_pointA.y - s_pointB.y) * s_normal.y;
				return seperation;
			}
			//default:
			//B2Settings.b2Assert(false);
			//return 0.0;
		}
	}
	
	
	public function new () {
		
		m_localPoint = new B2Vec2();
		m_axis = new B2Vec2();
		
	}
	
	public var m_proxyA:B2DistanceProxy;
	public var m_proxyB:B2DistanceProxy;
	public var m_type:B2SeparationFunctionType;
	public var m_localPoint:B2Vec2;
	public var m_axis:B2Vec2;
}