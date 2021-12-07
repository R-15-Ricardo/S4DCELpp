#include <vector>
#include <stack>
#include <set>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>

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

class line {
private:
	vector2f dir;
	vector2f origin;
	int type;

	friend intersection checkIntersection(line l1,line l2);

public:
	enum lineType {full = 0, segment = 1};
	line(vector2f origin_, vector2f dir_, std::string type) : dir(dir_), origin(origin_)
	{
		if (type == "full") this->type = lineType::full;
		else if (type == "segment") this->type = lineType::segment;
		else
			throw std::invalid_argument("Non-valid line type");
	}

	line(vector2f origin_, vector2f dir_) : dir(dir_), origin(origin_), type(lineType::segment) {};

	py::array IF_getDrawable() {
		return py::cast(std::vector<float>({origin.first,origin.second,dir.first,dir.second}));
	}

	py::tuple IF_getP1 () {
		return py::cast(this->origin);
	}

	void IF_setP1 (point p) {
		this->origin = p;
	}

	py::tuple IF_getP2 () {
		point p2 = this->origin + this->dir;
		return py::cast(p2);
	}

	void IF_setP2 (point p) {
		point newDir = p - this->origin;
		this->origin = newDir;
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


struct vertex;
struct face;
struct halfedge;

struct vertex {
	point pos;
	std::vector<halfedge*> incident;
};

struct face {
	point data = point({10000000000000000,10000000000000000});
	halfedge* leader = nullptr;
};

struct halfedge {
	halfedge* next = nullptr;
	halfedge* last = nullptr;
	halfedge* twin = nullptr;
 	vertex* origin = nullptr;
 	face* bounding = nullptr;
};

std::vector<halfedge*> getBoundry(face* f)
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

struct fullEdge {
	halfedge* h1;
	halfedge* h2;

	bool operator==(const fullEdge &other) const {
		return ((this->h1 == other.h1) && (this->h2 == other.h2) ||
				(this->h1 == other.h2) && (this->h2 == other.h1));
	}

	line IF_lineParamRep () {
		point o = h2->origin->pos;
		point dir = h1->origin->pos - o;
		return line(o, dir);
	}
};

struct vertexId {
	vertex* loc;

	bool operator==(const vertexId &other) const {
		return this->loc == other.loc;
	}

	py::tuple IF_pos () {
		return py::make_tuple(loc->pos.first,loc->pos.second);
	}

};

struct faceId {
	face* loc;

	bool operator==(const faceId &other) const {
		return this->loc == other.loc;
	}

	py::object IF_get_data() {
		if (this->loc->data == point({10000000000000000,10000000000000000}))
			return py::none();

		point a = this->loc->data;
		return py::cast(a);
	}

	void IF_set_data(point data_) {
		this->loc->data = data_;
	}

	std::vector<fullEdge> IF_boundry () {
		std::vector<fullEdge> out;

		halfedge* loopstart = this->loc->leader;
		halfedge* loopPtr = loopstart;

		do {
			out.push_back(fullEdge({loopPtr,loopPtr->twin}));
			loopPtr = loopPtr->next;
		} while (loopPtr != loopstart);

		return out;
	}
};

struct hedgeId {
	halfedge* loc;

	bool operator==(const hedgeId &other) const {
		return this->loc == other.loc;
	}

	hedgeId IF_next() {
		return hedgeId({this->loc->next});
	}

	hedgeId IF_last() {
		return hedgeId({this->loc->last});
	}

	hedgeId IF_twin() {
		return hedgeId({this->loc->twin});
	}

	faceId IF_bonding() {
		return faceId({this->loc->bounding});
	}
};

bool getWinding(face* f)
{
	halfedge* lead = f->leader;
	point A = lead->last->twin->origin->pos;
	point B = lead->last->origin->pos;
	point C = lead->origin->pos;

	vector2f v1 = A-B;
	vector2f v2 = C-B;

	return (aboveTest(v1,v2) > 0);
}

bool IF_testCWRotation (faceId fId)
{
	return getWinding(fId.loc);
}


class dcel {
	private:
		std::vector<vertex*> vertices;
		std::vector<face*> faces;
		std::vector<halfedge*> hedges;

		void buildFromGraph(std::vector<point> V, std::vector<edge> E);
		bool testInside(point p, face* f);
        
        vertex* VertexOnEdge(point x,  halfedge*);
        halfedge* splitFace(faceId fId, vertexId u, vertexId v);
        //std::vector<halfedge*> sortLines(std::vector<fullEdge>, point, int);
		face* getContainerFace(point x);
		face* mergeFaces(face* f1, face* f2, halfedge* link);
        void DeleteInsideNewFace(std::vector<fullEdge> boundNewFace, face* hint);

        //std::pair<faceId, faceId> facesTouchingLine(hedgeId);
        //vertexId isThisAVert(point);
        //bool areFacesEqual(faceId, faceId);

        template<class T>
        void purge(std::vector<T>& list, T item);

	public:

		dcel(std::vector<point> V, std::vector<edge> E) {
			buildFromGraph(V,E);
		}

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

		faceId IF_traverseFace(faceId fId, fullEdge throught)
		{
			face* f = fId.loc;
			return (throught.h1->bounding == f) ? faceId({throught.h2->bounding}) : faceId({throught.h1->bounding});
		}

		faceId IF_getFace (int index)
		{
			 if (index >= faces.size())
				 throw std::out_of_range("Face does not exist");

			 return faceId({faces[index]});
		}

		faceId IF_getLandingFace(point p)
		{
			return faceId({this->getContainerFace(p)});
		}

        vertexId IF_splitEdgeOnPoint(point x, fullEdge eId)
        {
            return vertexId({this->VertexOnEdge(x, eId.h1)});
        }

        fullEdge IF_splitFace(faceId fId, vertexId v, vertexId u)
        {
            //LOG("Vamos a partir una cara.");
			halfedge* piece = this->splitFace(fId,u,v);
            //LOG("Cara partida.");
            return fullEdge({piece,piece->twin});
        }

		void IF_createNewFace(std::vector<fullEdge> newBound, point p)
		{
			face* start = this->getContainerFace(p);
			start->data = p;
			this->DeleteInsideNewFace(newBound, start);
		}

		void IF_boundedMerge(std::vector<fullEdge> newBound, faceId fId)
		{
			this->DeleteInsideNewFace(newBound, fId.loc);
		}
};

void dcel::buildFromGraph(std::vector<point> V, std::vector<edge> E)
{
	//create vertices
	for (auto v : V)
	{
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


		h1->twin = h2;
		h2->twin = h1;

		hedges.emplace_back(h1);
		hedges.emplace_back(h2);
	}

	//link hedges
	for (auto he : hedges)
	{

		if (he->next != nullptr)
		{
			continue;
		}

		halfedge* loopPoint = he;
		halfedge* now = loopPoint;

		int circleSize = 0;
		do {
			vertex* nodehub = now->origin;
			halfedge* closest;
			float closestAngle = 1e300;
			float closetSide = -1;
		//  quick mafs to get least angle
		//  [----------------------------------
			point o = nodehub->pos;

			point a = now->twin->origin->pos;

			vector2f v1 = (1.0/abs(a-o))*(a-o);

			for (auto incident : nodehub->incident)
			{
				if (incident == now || incident->twin->last != nullptr) continue;

				point p = incident->twin->origin->pos;

				vector2f v2 = (1.0/abs(p-o))*(p-o);
				int side = aboveTest(v1,v2);

				float angle = ((-1*side)*dot(v1,v2) + 1)/2.0;




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

			//sleep(1);

			circleSize++;
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
		throw std::runtime_error("Couldn't identify faces correctly.");

}

bool dcel::testInside(point p, face* f)
{
	auto boundry = getBoundry(f);

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

// Operaciones con el DCEL
template<typename T>
void dcel::purge(std::vector<T>& list, T item)
{
    for (auto nowV = list.begin(); nowV != list.end(); nowV++)
        if(*nowV == item)
        {
			//delete item;
            list.erase(nowV);
            break;
        }
}

vertex* dcel::VertexOnEdge(point x, halfedge* e)
{
	//non-duplicate vertices
	if (abs(e->origin->pos - x) < 0.000000000001)
		return e->origin;

	if (abs(e->twin->origin->pos - x) < 0.000000000001)
		return e->twin->origin;

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
    this->purge(e->origin->incident, e);


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
    this->purge(e_twin->origin->incident, e_twin);


	new_hed_1->twin = twi_hed_2;
	new_hed_2->twin = twi_hed_1;
	twi_hed_1->twin = new_hed_2;
	twi_hed_2->twin = new_hed_1;

    if (e->bounding->leader == e)
        e->bounding->leader = new_hed_1;

    if (e_twin->bounding->leader == e_twin)
        e_twin->bounding->leader = twi_hed_1;

    this->purge(this->hedges, e);
    this->purge(this->hedges, e_twin);

	delete e;
	delete e_twin;

	return new_vert;
}


halfedge* dcel::splitFace(faceId fId, vertexId v1, vertexId v2)
{
	vertex* u = v1.loc;
	vertex* v = v2.loc;

	face* faceToDelete = fId.loc;
    //LOG("Inicializando.");

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
    //LOG("Ya inicializado.");

    face* f_1 = new face;      this->faces.emplace_back(f_1);
    face* f_2 = new face;      this->faces.emplace_back(f_2);
    halfedge* e_1 = new halfedge;  this->hedges.emplace_back(e_1);
    halfedge* e_2 = new halfedge;  this->hedges.emplace_back(e_2);

    //LOG("Comenzamos con la primera cara.");
    e_1->origin = v;
    e_2->origin = u;

    f_1->leader = e_1;
    f_2->leader = e_2;
    e_1->twin   = e_2;
    e_2->twin   = e_1;
    e_1->origin->incident.emplace_back(e_1);
    e_2->origin->incident.emplace_back(e_2);

    e_2->next = h_1->next;
    //LOG("Hay un core dump aqui :(.");
    e_2->next->last = e_2;
    //LOG("Ya no llegamos a esto.");
    e_2->last = h_2;

	halfedge* auxh_2next = h_2->next;
    e_2->last->next = e_2;

    //LOG("Matemos todas las referencias a la cara anterior.");
    halfedge* loopStart = e_2;
	halfedge* loopPtr = loopStart;
	do {
        loopPtr->bounding = f_2;
        loopPtr = loopPtr->next;
    } while (loopStart != loopPtr);


    //LOG("Continuemos con la segunda cara.");
    e_1->next = auxh_2next;
    e_1->next->last = e_1;
    e_1->last = h_1;
    e_1->last->next = e_1;

	loopStart = e_1;
	loopPtr = loopStart;
    //LOG("Matnado las referencias a la cara anterior nuevamente.");
	do {
		loopPtr->bounding = f_1;
		loopPtr = loopPtr->next;
	} while (loopStart != loopPtr);

    //LOG("Terminemos por matar la cara anterior.");
    this->purge(this->faces, faceToDelete);
	delete faceToDelete;

    //LOG("Hemos acabado.");
	return e_1;
}

face* dcel::getContainerFace(point x)
{
	face* unwind = nullptr;
	for (auto f : faces)
	{
		if (testInside(x,f)) return f;

		if (!getWinding(f)) unwind = f;
	}

	return unwind;
}

face* dcel::mergeFaces(face* f1, face* f2, halfedge* link)
{
	if (!((link->bounding == f1 && link->twin->bounding == f2) ||
		  (link->bounding == f2 && link->twin->bounding == f1))) {
		std::cout<<"Faces aren't touching"<<std::endl;
		return nullptr;
	}

	if (f1 == nullptr || f2 == nullptr) return nullptr;

	halfedge* loopStart = link->last;

	link->last->next = link->twin->next;
	link->twin->next->last = link->last;

	link->next->last = link->twin->last;
	link->twin->last->next = link->next;

	this->purge(link->twin->origin->incident, link->twin);
	this->purge(link->origin->incident, link);

	this->purge(this->hedges, link->twin);
	this->purge(this->hedges, link);

	delete link->twin;
	delete link;

	face* newFace = nullptr;

	if (f1 != f2)
	{
		newFace = new face;
		newFace->data = f1->data;

		this->purge(this->faces, f1);
		this->purge(this->faces, f2);

		delete f1;
		delete f2;

		this->faces.emplace_back(newFace);
	}
	else newFace = f1;

	newFace->leader = loopStart;
	loopStart->bounding = newFace;

	halfedge* loopPtr = loopStart->next;
	while (loopPtr != loopStart)
	{
		loopPtr->bounding = newFace;
		loopPtr = loopPtr->next;
	}

	return newFace;
}

void dcel::DeleteInsideNewFace(std::vector<fullEdge> boundNewFace, face* hint)
{
	std::stack<face*> marked;
	marked.push(hint);

	while (!marked.empty())
	{
		face* merger = marked.top();
		marked.pop();

		std::stack<halfedge*> toMerge;

		auto boundry = getBoundry(merger);
		for (auto e : boundry)
		{
			bool add = true;
			for (auto fe : boundNewFace)
			{
				if (fe.h1 == e || fe.h2 == e)
				{
					add = false;
					break;
				}
			}
			if (!getWinding(e->twin->bounding)) add = false;

			if (add) toMerge.push(e);
		}
		if (toMerge.empty()) break;

		while (!toMerge.empty())
		{
			halfedge* nowLink = toMerge.top();
			toMerge.pop();
			face* facePair = nowLink->twin->bounding;

			merger = this->mergeFaces(merger, facePair, nowLink);
		}

		if (merger != nullptr)
			marked.push(merger);
	}

	for (auto v : this->vertices)
	{
		if (v->incident.empty())
		{
			this->purge(this->vertices, v);
			delete v;
		}
	}
}



PYBIND11_MODULE(PyS4DCEL, handle) {
	handle.doc() = "Cpp DCEL module";
	handle.def("get_bisector", &IF_bisect);
	handle.def("get_intersection", &IF_intersect);
	handle.def("isCW",&IF_testCWRotation);

	py::class_<line>( handle, "line" )
	        .def(py::init<vector2f, vector2f, std::string>())
			.def_property("P1", &line::IF_getP1, &line::IF_setP1)
			.def_property("P2", &line::IF_getP2, &line::IF_setP2)
			.def_property_readonly("drawable", &line::IF_getDrawable);

	py::class_<dcel>( handle, "dcel" )
	        .def(py::init<std::vector<point>, std::vector<edge>>())
			.def("get_face", &dcel::IF_getFace)
			.def("step_over_edge", &dcel::IF_traverseFace)
			.def("landing_face", &dcel::IF_getLandingFace)
			.def("split_face", &dcel::IF_splitFace)
			.def("split_edge", &dcel::IF_splitEdgeOnPoint)
			.def("delete_interior", &dcel::IF_createNewFace)
			.def("delete_interior", &dcel::IF_boundedMerge)
			.def_property_readonly("G", &dcel::IF_getGraph);

	py::class_<vertexId>(handle, "vertexId")
	        .def(py::self == py::self)
			.def_property_readonly("pos", &vertexId::IF_pos);

	py::class_<hedgeId>(handle, "edgeId")
			.def(py::self == py::self)
			.def_property_readonly("next", &hedgeId::IF_next)
			.def_property_readonly("last", &hedgeId::IF_last)
			.def_property_readonly("twin", &hedgeId::IF_twin)
			.def_property_readonly("bounding", &hedgeId::IF_bonding);

	py::class_<faceId>(handle, "faceId")
			.def(py::self == py::self)
			.def_property_readonly("boundry", &faceId::IF_boundry)
	        .def_property("data", &faceId::IF_get_data, &faceId::IF_set_data);

	py::class_<fullEdge>(handle, "fullEdge")
	        .def(py::self == py::self)
			.def_property_readonly("line", &fullEdge::IF_lineParamRep);

}
