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
  [x] DCEL from graph constructor
  [x] DCEL default from two points
  [] `dcel::getFace(point p)` method implementation
  [] `dcel::getBoundry(face& f)` method implementation
  
# TODO *(pybind11 interface)*
  [] everyting...
