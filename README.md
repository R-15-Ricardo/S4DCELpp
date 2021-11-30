# S4DCELpp
DCEL Python module on cpp.

# Usage
Build with on extern directory with CMAKELists.txt. 
Add:
```python
import sys
sys.path.append("[repoPath]/S4DCELpp/cmake-build-debug")
```
to python source and import with `import PyS4DCEL`

# TODO *(cpp class)*:
  * [X] DCEL from graph constructor
  * [X] DCEL default from two points
  * [ ] Handle points to infinity
  * [X] `dcel::getFace(point p)` method implementation
  * [X] `dcel::getBoundry(face* f)` method implementation
  * [ ] `dcel::splitEdge(halfedge* e)` method implementation
  * [ ] `dcel::joinVertex(vertex* a, vertex* b)` method implementation
  * [ ] `dcel::splitFace(face* f)` method implementation
  * [X] Create line class
  * [ ] Line class intersect
  
# TODO *(pybind11 interface)*
  * [X] Constructors
  * [X] read-only property `vertices`
  * [X] read-only property `edges`
  * [X] read-only property `graph` *(for debug)*
  * [X] read-only property `faces`
