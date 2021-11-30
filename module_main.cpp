#include <vector>
#include <stack>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#define LOG(A) std::cout<<A<<std::endl

namespace py = pybind11;

#define INFTY -1

typedef std::pair<float,float> point;
typedef std::pair<float,float> vector2f;
typedef std::pair<int,int> edge;

point operator-(point a, point b) {
	return point(a.first-b.first, a.second-b.second);
}

point operator+(point a, point b) {
	return point(a.first+b.first, a.second+b.second);
}

template <class T>
point operator*(T x, point a) {
	return point(x*a.first, x*a.second);
}

float abs(vector2f p)
{
	return std::sqrt(p.first*p.first + p.second*p.second);
}

float dot(vector2f a, vector2f b)
{
	return a.first*b.first + a.second*b.second;
}

int aboveTest(vector2f a, vector2f b)
{
	return ((a.first*b.second - a.second*b.first) > 0) ? 1 : -1;
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

typedef std::vector<face*>::iterator faceIterator;

struct halfedge {
	halfedge* next = nullptr;
	halfedge* last = nullptr;
	halfedge* twin = nullptr;
 	vertex* origin = nullptr;
 	face* bounding = nullptr;
};

class line {
	private:
		vector2f dir = {1,0};
		vector2f origin = {0,0};
		int type = 0;

	public:
		enum lineType {full = 0, segment = 1};

		line();
		line(vector2f dir_) : dir(dir_), type(lineType::full) {};
		line(vector2f origin_, vector2f dir_) : dir(dir_), origin(origin_), type(lineType::segment) {};
		py::array IF_getDrawable() {
			return py::cast(std::vector<float>({origin.first,origin.second,dir.first,dir.second}));
		}

		friend py::array IF_intersect(line l1,line l2);
};

py::array IF_intersect(line l1, line l2)
{
	return py::cast(nullptr);
}

line IF_bisect(point a, point b)
{
	point mid = 0.5*(a + b);
	point dir = b - a;
	point dirRotate = {-dir.second, dir.first};

	return line(mid, dirRotate);
}

class dcel {
	private:
		std::vector<vertex*> vertices;
		std::vector<face*> faces;
		std::vector<halfedge*> hedges;

		void buildFromGraph(std::vector<point> V, std::vector<edge> E);
		void buildDefault(point a, point b);
		std::vector<halfedge*> getBoundry(face* f);
		bool testInside(point p, face* f);
		bool getWinding(face* f);

	public:

		dcel(std::vector<point> V, std::vector<edge> E) {
			buildFromGraph(V,E);
		}
//		dcel(point a, point b) {
//			this->buildDefault(a,b);
//		}

		//interface
		py::tuple IF_getGraph ()
		{
			std::vector<std::vector<float>> vertOut;
			for (auto v : vertices)
			{
				vertOut.push_back(std::vector<float>({v->pos.first,v->pos.second}));
			}

			std::vector<std::vector<float>> edgeOut;
			for (auto e : hedges)
			{
				point o = e->twin->origin->pos;
				point dir = e->origin->pos - o;
				edgeOut.push_back(std::vector<float>({o.first,o.second,dir.first,dir.second}));
			}
			py::array out1 = py::cast(vertOut);
			py::array out2 = py::cast(edgeOut);
			return py::make_tuple(out1,out2);
		}

		faceIterator IF_getFacesEnd ()
		{
			return this->faces.end();
		}

		faceIterator IF_getFace (int index)
		{
			 faceIterator id = faces.begin();
			 id += index;
			 if (id >= faces.end())
				 throw std::out_of_range("Face does not exist");

			 return id;
		}

		std::vector<line> IF_getFaceBoundry(faceIterator faceId)
		{
			auto boundry = this->getBoundry(*faceId);
			std::vector<line> hedgeOut;

			for (auto e : boundry)
			{
				point o = e->twin->origin->pos;
				point dir = e->origin->pos - o;
				hedgeOut.push_back(line(o,dir));
			}
			return hedgeOut;
		}

		faceIterator IF_getLandingFace(point p)
		{
			faceIterator unwind = faces.end();
			for (auto fid = faces.begin(); fid<faces.end(); fid++)
			{
				face* f = *fid;
				if (testInside(p,f)) return fid;

				if (!getWinding(f)) unwind = fid;
			}

			return unwind;
		}



};

void dcel::buildFromGraph(std::vector<point> V, std::vector<edge> E)
{
	//create vertices
	LOG("Creating vertices!");
	for (auto v : V)
	{
		LOG("("<<v.first<<","<<v.second<<")");
		vertices.emplace_back(new vertex{v});
	}

	//create hedges
	for (auto e : E)
	{
		halfedge* h1 = new halfedge;
		halfedge* h2 = new halfedge;

		h1->origin = vertices.at(e.second);
		h1->origin->incident.emplace_back(h1);
		h2->origin = vertices.at(e.first);
		h2->origin->incident.emplace_back(h2);

		LOG("h-e: ("<<h2->origin->pos.first<<","<<h2->origin->pos.second<<") -> ("<<h1->origin->pos.first<<","<<h1->origin->pos.second<<")");

		h1->twin = h2;
		h2->twin = h1;

		hedges.emplace_back(h1);
		hedges.emplace_back(h2);
	}

	//link hedges
	LOG("Creating hedges!");
	for (auto he : hedges)
	{
		LOG("Processing hedge: ("<<he->twin->origin->pos.first<<","<<he->twin->origin->pos.second<<") -> ("<<he->origin->pos.first<<","<<he->origin->pos.second<<")");

		if (he->next != nullptr)
		{
			LOG("STATE: LINKED.");
			continue;
		}

		LOG("STATE: LINKING...");
		halfedge* loopPoint = he;
		halfedge* now = loopPoint;

		int circleSize = 0;
		do {
			LOG("	FINDING CIRCLE STEP");
			vertex* nodehub = now->origin;
			halfedge* closest;
			float closestAngle = 1e300;
			float closetSide = -1;
		//  quick mafs to get least angle
		//  [----------------------------------
			point o = nodehub->pos;
			LOG("		NODE: ("<<o.first<<","<<o.second<<")");

			point a = now->twin->origin->pos;
			LOG("		T-NEIGHBOR: ("<<a.first<<","<<a.second<<")");

			vector2f v1 = (1.0/abs(a-o))*(a-o);

			for (auto incident : nodehub->incident)
			{
				if (incident == now || incident->twin->last != nullptr) continue;

				point p = incident->twin->origin->pos;
				LOG("		NEIGHBOR: ("<<p.first<<","<<p.second<<")");

				vector2f v2 = (1.0/abs(p-o))*(p-o);
				int side = aboveTest(v1,v2);
				LOG("			side: "<<side);

				float angle = ((-1*side)*dot(v1,v2) + 1)/2.0;
				LOG("			cos(Θ): "<<angle);




				if (side > closetSide)
				{
					closetSide = side;
					closestAngle = 1e300;
				}
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

			LOG("	-> ("<<now->origin->pos.first<<","<<now->origin->pos.second<<")");
			//sleep(1);

			circleSize++;
		} while (now != loopPoint);
		LOG("STATE: LINKING COMPLETED. LOOP SIZE "<<circleSize<<"\n");
	}

	//identify faces
	LOG("Creating faces!");
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

std::vector<halfedge*> dcel::getBoundry(face* f)
{
	std::vector<halfedge*> boundry;

	halfedge* loopstart = f->leader;
	boundry.push_back(loopstart);

	halfedge* loopPtr = loopstart->next;
	while (loopPtr != loopstart)
	{
		boundry.push_back(loopPtr);
		loopPtr = loopPtr->next;
	}

	return boundry;
}

bool dcel::testInside(point p, face* f)
{
	auto boundry = this->getBoundry(f);

	for (auto e : boundry)
	{
		point o = e->twin->origin->pos;
		point a = e->origin->pos;

		vector2f v1 = a-o;
		vector2f v2 = p-o;

		if (aboveTest(v1,v2) > 0)
			return false;

	}

	return true;
}

bool dcel::getWinding(face* f)
{
	halfedge* lead = f->leader;
	point A = lead->last->twin->origin->pos;
	point B = lead->last->origin->pos;
	point C = lead->origin->pos;

	vector2f v1 = A-B;
	vector2f v2 = C-B;

	return (aboveTest(v1,v2) > 0);
}


PYBIND11_MODULE(PyS4DCEL, handle) {
	handle.doc() = "Cpp DCEL module";
	handle.def("get_bisector", &IF_bisect);

	py::class_<line>( handle, "line" )
	        .def(py::init<vector2f, vector2f>())
			.def_property_readonly("drawable", &line::IF_getDrawable);

	py::class_<dcel>( handle, "dcel" )
	        .def(py::init<std::vector<point>, std::vector<edge>>())
			.def("get_face", &dcel::IF_getFace)
			.def("get_boundry", &dcel::IF_getFaceBoundry)
			.def("landing_face", &dcel::IF_getLandingFace)
			.def_property_readonly("G", &dcel::IF_getGraph)
			.def_property_readonly("no_face", &dcel::IF_getFacesEnd);

	py::class_<faceIterator>(handle, "faceId");
}