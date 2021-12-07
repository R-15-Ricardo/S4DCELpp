# S4DCELpp
DCEL Python module built with [pybind11](https://github.com/pybind/pybind11)

## Instalation
### Instaltion with pip
* Clone this repository and install with `pip install ./S4DCELpp`

or

* Download the [latest release](https://github.com/R-15-Ricardo/S4DCELpp/releases/tag/v0.2.1) and install with `pip install ./PyDCEL-[latest]`

### Build and instalation with cmake
* Clone this repository
* Create `build` directory
* `cmake [path to repo]`
* `make`

# Usage

On python src, import with 
```python
import PyS4DCEL
```

## Clases
* `PyS4DCEL.dcel`
* `PyS4DCEL.line`
* `PyS4DCEL.vertexId`
* `PyS4DCEL.edgeId`
* `PyS4DCEL.faceId`
* `PyS4DCEL.fullEdge`


## Functions
* `PyS4DCEL.get_bisector(l1 : tuple, l2 : tuple) -> PyS4DCEL.line`
* `PyS4DCEL.get_intersection(l1 : PyS4DCEL.line, l2 : PyS4DCEL.line) -> tuple`
* `PyS4DCEL.isCW(f : PyS4DCEL.faceId) -> bool`

# Example
```python
import PyS4DCEL as pdcl
import matplotlib.pyplot as plt

def draw_dcel(G: tuple()):
    for arrow in G[1]:
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(G[0].T[0],G[0].T[1])
    plt.show()

V = [(-4,4),(4,4),(4,-4),(-4,-4),(0,0)]
E = [(0,1),(1,2),(2,3),(3,0),(4,0),(4,1),(4,2),(4,3)]

myDcel = pdcl.dcel(V,E)
draw_dcel(myDcel.G)
```

