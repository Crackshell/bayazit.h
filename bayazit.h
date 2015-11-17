#pragma once

#include <algorithm>
#include <limits>
#include <vector>

// Max amount of vertices per polygon (defaults to 8 for Box2D)
#ifndef MAX_VERTICES
#define MAX_VERTICES 8
#endif

#ifndef EPSILON
#define EPSILON 0.0001f
#endif

namespace p2bayazit
{
	class Point
	{
	public:
		float x, y;

		Point() : x(0), y(0) { }
		Point(float x, float y) : x(x), y(y) { }
	};

	Point operator +(const Point &a, const Point &b) { return Point(a.x + b.x, a.y + b.y); }
	Point operator /(const Point &a, float x) { return Point(a.x / x, a.y / x); }

	typedef std::vector<Point> Polygon;

	class Decomposer
	{
	public:
		Decomposer() { }

		std::vector<Polygon> Decompose(const Polygon &vertices)
		{
			return TriangulatePolygon(vertices);
		}

	private:
		std::vector<Polygon> TriangulatePolygon(const Polygon &vertices)
		{
			std::vector<Polygon> list;
			Point lowerInt, upperInt;
			int lowerIndex = 0, upperIndex = 0;
			Polygon lowerPoly, upperPoly;

			for (int i = 0; i < (int)vertices.size(); ++i)
			{
				if (Reflex(i, vertices))
				{
					float upperDist;
					float lowerDist = upperDist = std::numeric_limits<float>().max();
					for (int j = 0; j < (int)vertices.size(); ++j)
					{
						// if line intersects with an edge
						float d;
						Point p;
						if (Left(At(i - 1, vertices), At(i, vertices), At(j, vertices)) && RightOn(At(i - 1, vertices), At(i, vertices), At(j - 1, vertices)))
						{
							// find the point of intersection
							p = LineIntersect(At(i - 1, vertices), At(i, vertices), At(j, vertices), At(j - 1, vertices));

							if (Right(At(i + 1, vertices), At(i, vertices), p))
							{
								// make sure it's inside the poly
								d = SquareDist(At(i, vertices), p);
								if (d < lowerDist)
								{
									// keep only the closest intersection
									lowerDist = d;
									lowerInt = p;
									lowerIndex = j;
								}
							}
						}

						if (Left(At(i + 1, vertices), At(i, vertices), At(j + 1, vertices)) && RightOn(At(i + 1, vertices), At(i, vertices), At(j, vertices)))
						{
							p = LineIntersect(At(i + 1, vertices), At(i, vertices), At(j, vertices), At(j + 1, vertices));

							if (Left(At(i - 1, vertices), At(i, vertices), p))
							{
								d = SquareDist(At(i, vertices), p);
								if (d < upperDist)
								{
									upperDist = d;
									upperIndex = j;
									upperInt = p;
								}
							}
						}
					}

					// if there are no vertices to connect to, choose a point in the middle
					if (lowerIndex == (upperIndex + 1) % (int)vertices.size())
					{
						Point p = ((lowerInt + upperInt) / 2.0f);

						Copy(i, upperIndex, vertices, lowerPoly);
						lowerPoly.push_back(p);
						Copy(lowerIndex, i, vertices, upperPoly);
						upperPoly.push_back(p);
					}
					else
					{
						double highestScore = 0, bestIndex = lowerIndex;
						while (upperIndex < lowerIndex)
							upperIndex += (int)vertices.size();

						for (int j = lowerIndex; j <= upperIndex; ++j)
						{
							if (CanSee(i, j, vertices))
							{
								double score = 1.0f / (SquareDist(At(i, vertices), At(j, vertices)) + 1);
								if (Reflex(j, vertices))
								{
									if (RightOn(At(j - 1, vertices), At(j, vertices), At(i, vertices)) && LeftOn(At(j + 1, vertices), At(j, vertices), At(i, vertices)))
										score += 3;
									else
										score += 2;
								}
								else
								{
									score += 1;
								}
								if (score > highestScore)
								{
									bestIndex = j;
									highestScore = score;
								}
							}
						}
						Copy(i, (int)bestIndex, vertices, lowerPoly);
						Copy((int)bestIndex, i, vertices, upperPoly);
					}

					auto lower = TriangulatePolygon(lowerPoly);
					for (auto p : lower)
						list.push_back(p);

					auto upper = TriangulatePolygon(upperPoly);
					for (auto p : upper)
						list.push_back(p);

					return list;
				}
			}

			// polygon is already convex
			if ((int)vertices.size() > MAX_VERTICES)
			{
				Copy(0, (int)vertices.size() / 2, vertices, lowerPoly);
				Copy((int)vertices.size() / 2, 0, vertices, upperPoly);

				auto lower = TriangulatePolygon(lowerPoly);
				for (auto p : lower)
					list.push_back(p);

				auto upper = TriangulatePolygon(upperPoly);
				for (auto p : upper)
					list.push_back(p);
			}
			else
				list.push_back(vertices);

			return list;
		}

		Point At(int i, const Polygon &vertices)
		{
			int s = (int)vertices.size();
			return vertices[i < 0 ? s - 1 - ((-i - 1) % s) : i % s];
		}

		bool Reflex(int i, const Polygon &vertices)
		{
			return Right(i, vertices);
		}

		bool Left(const Point &a, const Point &b, const Point &c)
		{
			return Area(a, b, c) > 0;
		}

		bool LeftOn(const Point &a, const Point &b, const Point &c)
		{
			return Area(a, b, c) >= 0;
		}

		bool Right(int i, const Polygon &vertices)
		{
			return Right(At(i - 1, vertices), At(i, vertices), At(i + 1, vertices));
		}

		bool Right(const Point &a, const Point &b, const Point &c)
		{
			return Area(a, b, c) < 0;
		}

		bool RightOn(const Point &a, const Point &b, const Point &c)
		{
			return Area(a, b, c) <= 0;
		}

		float Area(const Point &a, const Point &b, const Point &c)
		{
			return (((b.x - a.x)*(c.y - a.y)) - ((c.x - a.x)*(b.y - a.y)));
		}

		float SquareDist(const Point &a, const Point &b)
		{
			float dx = b.x - a.x;
			float dy = b.y - a.y;
			return dx * dx + dy * dy;
		}

		bool CanSee(int i, int j, const Polygon &vertices)
		{
			if (Reflex(i, vertices))
			{
				if (LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)) && RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)))
					return false;
			}
			else
			{
				if (RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)) || LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)))
					return false;
			}
			if (Reflex(j, vertices))
			{
				if (LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)) && RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)))
					return false;
			}
			else
			{
				if (RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)) || LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)))
					return false;
			}
			for (int k = 0; k < (int)vertices.size(); ++k)
			{
				if ((k + 1) % (int)vertices.size() == i || k == i || (k + 1) % (int)vertices.size() == j || k == j)
					continue; // ignore incident edges

				Point intersectionPoint;

				if (LineIntersect(At(i, vertices), At(j, vertices), At(k, vertices), At(k + 1, vertices), intersectionPoint))
					return false;
			}
			return true;
		}

		void Copy(int i, int j, const Polygon &vertices, Polygon &to)
		{
			to.clear();

			while (j < i)
				j += vertices.size();

			for (; i <= j; ++i) {
				to.push_back(At(i, vertices));
			}
		}

		Point LineIntersect(const Point &p1, const Point &p2, const Point &q1, const Point &q2)
		{
			Point i;
			float a1 = p2.y - p1.y;
			float b1 = p1.x - p2.x;
			float c1 = a1 * p1.x + b1 * p1.y;
			float a2 = q2.y - q1.y;
			float b2 = q1.x - q2.x;
			float c2 = a2 * q1.x + b2 * q1.y;
			float det = a1 * b2 - a2 * b1;

			if (!FloatEquals(det, 0)) {
				// lines are not parallel
				i.x = (b2 * c1 - b1 * c2) / det;
				i.y = (a1 * c2 - a2 * c1) / det;
			}
			return i;
		}

		bool LineIntersect(const Point &point1, const Point &point2, const Point &point3, const Point &point4, bool firstIsSegment, bool secondIsSegment, Point &point)
		{
			point.x = 0;
			point.y = 0;

			// these are reused later.
			// each lettered sub-calculation is used twice, except
			// for b and d, which are used 3 times
			float a = point4.y - point3.y;
			float b = point2.x - point1.x;
			float c = point4.x - point3.x;
			float d = point2.y - point1.y;

			// denominator to solution of linear system
			float denom = (a * b) - (c * d);

			// if denominator is 0, then lines are parallel
			if (!(denom >= -EPSILON && denom <= EPSILON))
			{
				float e = point1.y - point3.y;
				float f = point1.x - point3.x;
				float oneOverDenom = 1.0f / denom;

				// numerator of first equation
				float ua = (c * e) - (a * f);
				ua *= oneOverDenom;

				// check if intersection point of the two lines is on line segment 1
				if (!firstIsSegment || ua >= 0.0f && ua <= 1.0f)
				{
					// numerator of second equation
					float ub = (b * e) - (d * f);
					ub *= oneOverDenom;

					// check if intersection point of the two lines is on line segment 2
					// means the line segments intersect, since we know it is on
					// segment 1 as well.
					if (!secondIsSegment || ub >= 0.0f && ub <= 1.0f)
					{
						// check if they are coincident (no collision in this case)
						if (ua != 0.0f || ub != 0.0f)
						{
							//There is an intersection
							point.x = point1.x + ua * b;
							point.y = point1.y + ua * d;
							return true;
						}
					}
				}
			}

			return false;
		}

		bool LineIntersect(const Point &point1, const Point &point2, const Point &point3, const Point &point4, Point &intersectionPoint)
		{
			return LineIntersect(point1, point2, point3, point4, true, true, intersectionPoint);
		}

		bool FloatEquals(float value1, float value2)
		{
			return fabs(value1 - value2) <= EPSILON;
		}
	};
}
