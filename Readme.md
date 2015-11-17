# Bayazit.h
This is a polygon decomposer using the [Bayazit algorithm](http://mpen.ca/406/bayazit). It is
ported from the [Farseer Physics](https://farseerphysics.codeplex.com/) C# code base into a
single-header file library.

Exmaple usage:

```C++
Bayazit::Polygon poly;
poly.push_back(Bayazit::Point(20, -30));
poly.push_back(Bayazit::Point(10, -10));
poly.push_back(Bayazit::Point(30, -20));
poly.push_back(Bayazit::Point(30, 20));
poly.push_back(Bayazit::Point(10, 10));
poly.push_back(Bayazit::Point(20, 30));
poly.push_back(Bayazit::Point(-20, 30));
poly.push_back(Bayazit::Point(-10, 10));
poly.push_back(Bayazit::Point(-30, 20));
poly.push_back(Bayazit::Point(-30, -20));
poly.push_back(Bayazit::Point(-10, -10));
poly.push_back(Bayazit::Point(-20, -30));

Bayazit::Decomposer dec;
auto polys = dec.Decompose(poly);

for (auto p : polys)
{
	printf("poly: (%d points)\n", p.size());
	for (auto v : p)
		printf("  %f %f\n", v.x, v.y);
}
```
