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
  * [ ] `dcel::getFace(point p)` method implementation
  * [ ] `dcel::getBoundry(face& f)` method implementation
  
# TODO *(pybind11 interface)*
  * [X] Constructors
  * [ ] read-only property `vertices`
  * [ ] read-only property `edges`
  * [X] read-only property `graph` *(for debug)*
  * [ ] read-only property `faces`
