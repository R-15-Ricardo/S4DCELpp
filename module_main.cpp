#include <vector>
#include <stack>
#include <set>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#define LOG(A) std::cout<<A<<std::endl

namespace py = pybind11;

#define INFTY -1

typedef std::pair<float,float> point;
typedef std::pair<float,float> vector2f;

typedef std::pair<bool, point> intersection;

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

float cross(vector2f a, vector2f b)
{
	return (a.first*b.second - a.second*b.first);
}

int aboveTest(vector2f a, vector2f b)
{
	return (cross(a,b) >= 0) ? 1 : -1;
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

struct vertexId {
	vertex* loc;
};

struct faceId {
	face* loc;
};

struct hedgeId {
	halfedge* loc;
};

class line {
	private:
		vector2f dir;
		vector2f origin;
		int type;
		hedgeId boundryId;

		friend intersection checkIntersection(line l1,line l2);
		friend line getBisector (point a, point b);

	public:
		enum lineType {full = 0, segment = 1};
		line(vector2f origin_, vector2f dir_, std::string type) : dir(dir_), origin(origin_), boundryId({nullptr})
		{
			if (type == "full") this->type = lineType::full;
			else if (type == "segment") this->type = lineType::segment;
			else
				throw std::invalid_argument("Non-valid line type");
		}

		line(vector2f origin_, vector2f dir_, hedgeId boundId) : dir(dir_), origin(origin_), type(lineType::segment), boundryId(boundId) {};

		py::array IF_getDrawable() {
			return py::cast(std::vector<float>({origin.first,origin.second,dir.first,dir.second}));
		}

		hedgeId IF_getHedgeId() {
			return this->boundryId;
		}

		friend py::object IF_intersect(line l1,line l2);
};


intersection checkIntersection (line l1, line l2)
{
	vector2f A = l2.origin - l1.origin;
	float B = cross(l1.dir, l2.dir);

	if (B == 0)
		return intersection({false, point()});

	float tl1 = cross(A,l2.dir)/B;
	float tl2 = cross(A,l1.dir)/B;

	// line - segment
	point inter;

	if (l1.type != l2.type)
	{
		int seg = (l1.type == line::lineType::segment) ? 1 : 2;
		inter = l1.origin + tl1*l1.dir;

		switch (seg) {
			case 1:
				if (0 <= tl1 && tl1 <= 1)
					return intersection ({true,inter});

			case 2:
				if (0 <= tl2 && tl2 <= 1)
					return intersection ({true,inter});
		}
		return intersection({false,point()});
	}
	else
	{
		switch (l1.type) {
			//line - line
			case line::lineType::full:
				inter = l1.origin + tl1*l1.dir;
				return intersection ({true,inter});

				//segment - segment
			case line::lineType::segment:
				if ((0<=tl1 && tl1<=1) && (0<=tl2 && tl2<=1))
				{
					inter = l1.origin + tl1*l1.dir;
					return intersection ({true,inter});
				}
				return intersection({false,point()});
		}
	}

	return intersection({false,point()});
}

line getBisector (point a, point b)
{
	point mid = 0.5*(a + b);
	point dir = b - a;
	point dirRotate = {-dir.second, dir.first};

	return line(mid, dirRotate, "full");
}

py::object IF_intersect(line l1, line l2)
{
	intersection interPoint = checkIntersection(l1,l2);

	if (!interPoint.first)
		return py::none();

	py::array arrOut = py::cast(std::vector<float>({interPoint.second.first, interPoint.second.second}));
	return arrOut;
}

line IF_bisect(point a, point b)
{
	return getBisector(a,b);
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
        
        vertex* VertexOnEdge(point x,  hedgeId eId);
        void splitFace(faceId fId, vertexId u, vertexId v);
        void DeleteInsideNewFace(std::vector<halfedge*> boundNewFace);

        std::pair<faceId, faceId> facesTouchingLine(hedgeId);
        vertexId isThisAVert(point);

        template<class T>
        void RemoveItemFromVec(std::vector<T>& list, T item);

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

		faceId IF_getFace (int index)
		{
			 if (index >= faces.size())
				 throw std::out_of_range("Face does not exist");

			 return faceId({faces[index]});
		}

		std::vector<line> IF_getFaceBoundry(faceId faceId)
		{
			auto boundry = this->getBoundry(faceId.loc);
			std::vector<line> hedgeOut;

			for (auto e : boundry)
			{
				point o = e->twin->origin->pos;
				point dir = e->origin->pos - o;
				hedgeOut.push_back(line(o,dir,hedgeId({e})));
			}
			return hedgeOut;
		}

		faceId IF_getLandingFace(point p)
		{
			faceId unwind = {nullptr};
			for (auto f : faces)
			{
				if (testInside(p,f)) return faceId({f});

				if (!getWinding(f)) unwind.loc = f;
			}

			return unwind;
		}

        vertexId IF_splitEdgeOnPoint(point x, hedgeId eId)
        {
            return vertexId({this->VertexOnEdge(x, eId)});
        }

        void IF_splitFace(faceId fId, vertexId v, vertexId u)
        {
            this->splitFace(fId,u,v);
        }

        void IF_deleteInsideNewFace(std::vector<line> boundry)
        {
			std::vector<halfedge*> boundNewFace;
			for (auto e : boundry)
			{
				halfedge* he = e.IF_getHedgeId().loc;
				boundNewFace.push_back(he);
			}
            this->DeleteInsideNewFace(boundNewFace);
        }

        py::tuple IF_facesTouchingLine(hedgeId h)
        {
            std::pair<faceId, faceId> result = facesTouchingLine(h);
            return py::make_tuple(result.first, result.second);
        }

        py::tuple IF_isThisAVert(float x, float y)
        {
            vertexId result = this->isThisAVert(point({x, y}));

            return py::make_tuple(result.loc == nullptr ? false : true, result);
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
	{
		std::cout<<"Fatal error on creation of faces!!!"<<std::endl;
		throw std::runtime_error("Couldn't identify faces correctly.");
	}

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

// Operaciones con el DCEL
template<typename T>
void dcel::RemoveItemFromVec(std::vector<T>& list, T item)
{
    for (auto nowV = list.begin(); nowV != list.end(); nowV++)
        if(*nowV == item)
        {
			//delete item;
            list.erase(nowV);
            break;
        }
}

vertex* dcel::VertexOnEdge(point x, hedgeId eId)
{
	halfedge* e = eId.loc;

    vertex* new_vert = new vertex;    this->vertices.emplace_back(new_vert);
    halfedge* new_hed_1 = new halfedge; this->hedges.emplace_back(new_hed_1);
    halfedge* new_hed_2 = new halfedge; this->hedges.emplace_back(new_hed_2);

    new_vert->pos = x;
    new_hed_1->origin = new_vert;
    new_hed_2->origin = e->origin;

    new_vert->incident.emplace_back(new_hed_1);

    new_hed_2->next     = e->next;
    new_hed_2->last     = new_hed_1;
    new_hed_2->bounding = e->bounding;

    new_hed_1->next     = new_hed_2;
    new_hed_1->last     = e->last;
    new_hed_1->bounding = e->bounding;

    e->last->next = new_hed_1;
    e->next->last = new_hed_2;

	new_hed_2->origin->incident.emplace_back(new_hed_2);
    this->RemoveItemFromVec(e->origin->incident, e);


    halfedge* e_twin = e->twin;
    halfedge* twi_hed_1 = new halfedge;    this->hedges.emplace_back(twi_hed_1);
    halfedge* twi_hed_2 = new halfedge;    this->hedges.emplace_back(twi_hed_2);

    twi_hed_1->origin = new_vert;
    twi_hed_2->origin = e_twin->origin;

    new_vert->incident.emplace_back(twi_hed_1);

    twi_hed_2->next     = e_twin->next;
    twi_hed_2->last     = twi_hed_1;
    twi_hed_2->bounding = e_twin->bounding;

    twi_hed_1->next     = twi_hed_2;
    twi_hed_1->last     = e_twin->last;
    twi_hed_1->bounding = e_twin->bounding;

    e_twin->last->next = twi_hed_1;
    e_twin->next->last = twi_hed_2;

	twi_hed_2->origin->incident.emplace_back(twi_hed_2);
    this->RemoveItemFromVec(e_twin->origin->incident, e_twin);


	new_hed_1->twin = twi_hed_2;
	new_hed_2->twin = twi_hed_1;
	twi_hed_1->twin = new_hed_2;
	twi_hed_2->twin = new_hed_1;

    if (e->bounding->leader == e)
        e->bounding->leader = new_hed_1;

    if (e_twin->bounding->leader == e_twin)
        e_twin->bounding->leader = twi_hed_1;

    this->RemoveItemFromVec(this->hedges, e);
    this->RemoveItemFromVec(this->hedges, e_twin);

	delete e;
	delete e_twin;

	return new_vert;
}


void dcel::splitFace(faceId fId, vertexId v1, vertexId v2)
{
	vertex* u = v1.loc;
	vertex* v = v2.loc;

	face* faceToDelete = fId.loc;

	halfedge* h_1;
	for (auto incd : u->incident)
	{
		if (incd->bounding == faceToDelete)
		{
			h_1 = incd;
			break;
		}
	}
	halfedge* h_2;
	for (auto incd : v->incident)
	{
		if (incd->bounding == faceToDelete)
		{
			h_2 = incd;
			break;
		}
	}

    face* f_1 = new face;      this->faces.emplace_back(f_1);
    face* f_2 = new face;      this->faces.emplace_back(f_2);
    halfedge* e_1 = new halfedge;  this->hedges.emplace_back(e_1);
    halfedge* e_2 = new halfedge;  this->hedges.emplace_back(e_2);
    //vertex* u = h_1->origin;
    //vertex* v = h_2->origin;
    //face* faceToDelete = h_1->bounding;

    e_1->origin = v;
    e_2->origin = u;

    f_1->leader = e_1;
    f_2->leader = e_2;
    e_1->twin   = e_2;
    e_2->twin   = e_1;
    e_1->origin->incident.emplace_back(e_1);
    e_2->origin->incident.emplace_back(e_2);

    e_2->next = h_1->next;
    e_2->next->last = e_2;
    e_2->last = h_2;

	halfedge* auxh_2next = h_2->next;
    e_2->last->next = e_2;

    halfedge* loopStart = e_2;
	halfedge* loopPtr = loopStart;
	do {
        loopPtr->bounding = f_2;
        loopPtr = loopPtr->next;
    } while (loopStart != loopPtr);


    e_1->next = auxh_2next;
    e_1->next->last = e_1;
    e_1->last = h_1;
    e_1->last->next = e_1;

	loopStart = e_1;
	loopPtr = loopStart;
	do {
		loopPtr->bounding = f_1;
		loopPtr = loopPtr->next;
	} while (loopStart != loopPtr);

    this->RemoveItemFromVec(this->faces, faceToDelete);
	delete faceToDelete;
}

void dcel::DeleteInsideNewFace(std::vector<halfedge*> boundNewFace)
{
    face* newFace;      this->faces.emplace_back(newFace);
    newFace->leader = boundNewFace.at(0);

    size_t n = boundNewFace.size();

    boundNewFace.at(0)->last = boundNewFace.at(n - 1);
    boundNewFace.at(0)->next = boundNewFace.at(1);
    for (size_t i = 1; i < n - 1; i++)
    {
        boundNewFace.at(i)->last = boundNewFace.at(i - 1);
        boundNewFace.at(i)->next = boundNewFace.at(i + 1);
    }
    boundNewFace.at(n - 1)->last = boundNewFace.at(n - 2);
    boundNewFace.at(n - 1)->next = boundNewFace.at(0);

    std::vector<vertex*> vertInsideFace;
    for (auto v : this->vertices)
        if (this->testInside(v->pos, newFace))
            vertInsideFace.push_back(v);
    // TODO: Creo que lo anterior también inserta los vértices de la nueva cara
    // :((.

    std::vector<halfedge*> hedgeToDelete;
    std::set<face*> facesToDelete;
    for (auto v : vertInsideFace)
        for (auto h : v->incident)
        {
            hedgeToDelete.push_back(h);
            facesToDelete.insert(h->bounding);
        }

    for (auto vertToDel : vertInsideFace)
    {
        this->RemoveItemFromVec(this->vertices, vertToDel);
		delete vertToDel;
    }
    for (auto hedgToDel : hedgeToDelete)
    {
        this->RemoveItemFromVec(this->hedges, hedgToDel);
		delete hedgToDel;
    }
    for (auto faceToDel : facesToDelete)
    {
        this->RemoveItemFromVec(this->faces, faceToDel);
		delete faceToDel;
    }

    for (auto newHedg : boundNewFace)
    {
        newHedg->bounding = newFace;
    }
}

std::pair<faceId, faceId> dcel::facesTouchingLine(hedgeId h)
{
    halfedge* e = h.loc;

    return std::pair<faceId, faceId> (faceId({e->bounding}), faceId({e->twin->bounding}));
}

vertexId dcel::isThisAVert(point p)
{
    float x, y, dist;
    for (auto v : this->vertices)
    {
        x    = v->pos.first  - p.first;
        y    = v->pos.second - p.second;
        dist = sqrt(x*x + y*y);

        if (dist < 1e-4)
            return vertexId({v});
    }
    return vertexId({nullptr});
}


PYBIND11_MODULE(PyS4DCEL, handle) {
	handle.doc() = "Cpp DCEL module";
	handle.def("get_bisector", &IF_bisect);
	handle.def("get_intersection", &IF_intersect);

	py::class_<line>( handle, "line" )
	        .def(py::init<vector2f, vector2f, std::string>())
			.def_property_readonly("drawable", &line::IF_getDrawable)
			.def_property_readonly("on_bound_id", &line::IF_getHedgeId);

	py::class_<dcel>( handle, "dcel" )
	        .def(py::init<std::vector<point>, std::vector<edge>>())
			.def("get_face", &dcel::IF_getFace)
			.def("get_boundry", &dcel::IF_getFaceBoundry)
			.def("landing_face", &dcel::IF_getLandingFace)
			.def("split_edge", &dcel::IF_splitEdgeOnPoint)
			.def("split_face", &dcel::IF_splitFace)
			.def("delete_interior", &dcel::IF_deleteInsideNewFace)
			.def_property_readonly("G", &dcel::IF_getGraph)
            .def("faces_touch_line", &dcel::IF_facesTouchingLine)
            .def("is_a_vert", &dcel::IF_isThisAVert);

	py::class_<faceId>(handle, "faceId");
	py::class_<hedgeId>(handle, "edgeId");
	py::class_<vertexId>(handle, "vertexId");
}
