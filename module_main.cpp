#include <vector>
#include <stack>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <iostream>

namespace py = pybind11;

#define INFTY -1

typedef std::pair<float,float> point;
typedef std::pair<float,float> vector2f;
typedef std::pair<int,int> edge;

point operator-(point a, point b) {
	return point(a.first-b.first, a.second-b.second);
}

template <class T>
point operator*(T x, point a) {
	return point(x*a.first, x*a.second);
}

float abs(vector2f p)
{
	return sqrt(p.first*p.first + p.second*p.second);
}

float dot(vector2f a, vector2f b)
{
	return a.first*b.first + a.second*b.second;
}


struct vertex;
struct face;
struct halfedge;

struct vertex {
	point pos;
	std::vector<halfedge*> incident;
};

struct face {
	point data;
	halfedge* leader = nullptr;
};

struct halfedge {
	halfedge* next = nullptr;
	halfedge* last = nullptr;
	halfedge* twin = nullptr;
 	vertex* origin = nullptr;
 	face* bounding = nullptr;
};

class dcel {
	private:
		std::vector<vertex*> vertices;
		std::vector<face*> faces;
		std::vector<halfedge*> hedges;

		void buildFromGraph(std::vector<point> V, std::vector<edge> E);
		void buildDefault(point a, point b);

	public:
		dcel(std::vector<point> V, std::vector<edge> E) {
			this->buildFromGraph(V,E);
		}
		dcel(point a, point b) {
			this->buildDefault(a,b);
		}

		//interface
		py::tuple IF_getGraph ()
		{
			std::vector<point> vertOut;
			for (auto v : vertices)
			{
				vertOut.push_back(v->pos);
			}

			auto edgeOut = std::vector<std::vector<point>>();
			for (auto e : hedges)
				edgeOut.push_back(std::vector<point>({e->origin->pos,e->twin->origin->pos}));

			return py::make_tuple(vertOut,edgeOut);
		}

};

void dcel::buildFromGraph(std::vector<point> V, std::vector<edge> E)
{
	//create vertices
	for (auto v : V)
		vertices.emplace_back(new vertex{v});

	//create hedges
	for (auto e : E)
	{
		halfedge* h1 = new halfedge;
		halfedge* h2 = new halfedge;
		h1->origin = vertices.at(e.second);
		h1->origin->incident.emplace_back(h1);
		h2->origin = vertices.at(e.first);
		h2->origin->incident.emplace_back(h2);

		h1->twin = h2;
		h2->twin = h1;

		hedges.emplace_back(h1);
		hedges.emplace_back(h2);
	}

	//link hedges
	for (auto he : hedges)
	{
		halfedge* loopPoint = he;
		halfedge* now = loopPoint;

		do {
			vertex* nodehub = now->origin;
			halfedge* closest;
			float closestAngle = 1e300;

		//  quick mafs to get least angle
		//  [----------------------------------
			point o = nodehub->pos;
			point a = now->twin->origin->pos;
			vector2f v1 = (1.0/abs(a-o))*a-o;

			for (auto incident : nodehub->incident)
			{
				point p = incident->twin->origin->pos;
				vector2f v2 = (1.0/abs(p-o))*p-o;
				float angle = dot(v1,v2);

				if (angle < closestAngle)
				{
					closestAngle = angle;
					closest = incident;
				}
			}
		//	----------------------------------]

			now->next = closest->twin;
			now->next->last = now;

			now = now->next;

		} while (now != loopPoint);
	}

	//identify faces
	std::stack<halfedge*> tracer;
	tracer.push(hedges.front());
	while (!tracer.empty())
	{
		halfedge* now = tracer.top();
		tracer.pop();

		if (now->bounding == nullptr)
		{
			//circle the face and indentify
			face* nFace = new face;
			nFace->leader = now;
			now->bounding = nFace;
			tracer.push(now->twin);

			faces.emplace_back(nFace);

			halfedge* loopPoint = now;
			halfedge* place = loopPoint->next;
			while (place != loopPoint)
			{
				place->bounding = nFace;
				tracer.push(place->twin);
				place = place->next;
			}
		}
	}

	//sanity check
	if (2 - vertices.size() + (hedges.size()/2) != faces.size())
		std::cout<<"Fatal error on creation of faces!!!"<<std::endl;

}

void dcel::buildDefault(point a, point b)
{
	std::vector<point> points = {a,b};
	std::vector<edge> edges = {edge(INFTY,INFTY)};
}


PYBIND11_MODULE(PyS4DCEL, handle) {
	handle.doc() = "Cpp DCEL module";

	py::class_<dcel>( handle, "dcel" )
	        .def(py::init<std::vector<point>, std::vector<edge>>())
	        .def(py::init<point,point>());
}