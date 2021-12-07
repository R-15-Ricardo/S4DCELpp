import numpy as np
import sys

from typing import List
import matplotlib.pyplot as plt

import PyS4DCEL as pdcl

def draw_voronoi(G: tuple()):
    for arrow in G[1]:
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(G[0].T[0],G[0].T[1])
    plt.scatter(sites.T[0],sites.T[1])
    plt.show()

def draw_face(G: tuple(), face):
    bound = face.boundry

    for edge in bound:
        arrow = edge.line.drawable
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(G[0].T[0],G[0].T[1])
    plt.scatter(sites.T[0],sites.T[1])
    plt.show()

def draw_bisector(G: tuple(), line, new_site, landing_site):
    for arrow in G[1]:
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(G[0].T[0],G[0].T[1])
    plt.scatter(sites.T[0],sites.T[1])
    plt.scatter(new_site[0],new_site[1],color="magenta")
    plt.scatter(landing_site[0],landing_site[1],color="green")

    arrow = line.drawable
    plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.00001,length_includes_head=True,shape='left')

    plt.show()

def draw_intersection(G: tuple(), face, intersect):
    bound = face.boundry

    for edge in bound:
        arrow = edge.line.drawable
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(G[0].T[0],G[0].T[1])
    plt.scatter(sites.T[0],sites.T[1])
    plt.scatter(intersect[0],intersect[1],color="red")

    plt.show()



def xtractArguments(args : List[str]):
    if len(args) < 2:
        print(f"\nUsa el programa de la siguiente manera:\n\t./{args[0]} <sites.txt>")
        sys.exit(1)

    sites = np.loadtxt(args[1])
    if sites.shape[1] != 2:
        print("\nEl archivo no usa coordenadas 2D.")
        sys.exit(1)

    norm_sites = sites / np.abs(sites).max(axis=0)
    return norm_sites



def main(sites: np.ndarray):
    # Empezamos definiendo el marco exterior.
    V = [(-1.2, -1.2), (-1.2,  1.2), ( 1.2,  1.2), ( 1.2, -1.2)]
    E = [(0, 1), (1, 2), (2, 3), (3, 0)]
    ourVoronoi = pdcl.dcel(V, E)

    draw_voronoi(ourVoronoi.G)

    canvas = ourVoronoi.get_face(0)
    canvasBound = canvas.boundry

    # Supondremos que el primer sitio le pertenece TODA la cara del marco.
    initLine = pdcl.get_bisector(tuple(sites[0]),tuple(sites[1]))

    to_join = []

    for edge in canvasBound:
        inter = pdcl.get_intersection(initLine,edge.line)
        if inter is not None:
            to_join.append(ourVoronoi.split_edge(tuple(inter),edge))
    ourVoronoi.split_face(canvas,to_join[0],to_join[1])

    div1 = ourVoronoi.landing_face(sites[0])
    div2 = ourVoronoi.landing_face(sites[1])
    div1.data = sites[0]
    div2.data = sites[1]

    draw_voronoi(ourVoronoi.G)

    #return ourVoronoi.G

    # Por cada nuevo sitio, actualizaremos el diagrama de Voronoi.
    for i in range(2, len(sites)):
        new_site        = sites[i]
        faces_to_divide = list()

        # Conforme encntremos las intersecciones usando bisectores, guardaremos
        # las caras vecinas. Esto lo repetiremos hasta que las intersecciones
        # dejen de tocar una arista que toca una cara no explorada.
        facesToExplore = list()
        facesExplored = list()


        # Siempre comenzamos con la cara en la que cae nuestro nuevo sitio.
        facesToExplore.append(ourVoronoi.landing_face(new_site))

        print("circleing")

        shift = None

        while facesToExplore:
            fid = facesToExplore.pop()
            landing_site = fid.data

            if landing_site is None:
                continue

            # Podemos calcular el bisector y guardaremos los datos para, más
            # adelante, poder dividir la cara.
            bisector = pdcl.get_bisector(new_site, landing_site)

            if shift is not None:
                bisector.P1 = shift

            draw_bisector(ourVoronoi.G, bisector, new_site, landing_site)
            bound    = fid.boundry
            all_intersec = []
            intersec = []
            to_split = []
            for edge in bound:
                aux = pdcl.get_intersection(bisector, edge.line)

                if aux is not None:
                    draw_intersection(ourVoronoi.G, fid, aux)
                    intersec.append(aux)
                    all_intersec.append(aux)
                    to_split.append(edge)

                    next = ourVoronoi.step_over_edge(fid, edge)

                    if pdcl.isCW(next) and aux not in all_intersec:
                        shift = tuple(aux)
                        facesToExplore.append(next)


            faces_to_divide.append((fid,intersec[:], to_split[:]))
            shift = None

        # Ahora haremos todas las divisiones de todas las caras

        print("slicing")
        new_face = []
        for f, inter, split in faces_to_divide:
            to_join = []

            for i in range(len(split)):
                to_join.append(ourVoronoi.split_edge(tuple(inter[i]), split[i]))
                draw_voronoi(ourVoronoi.G)

            new_face.append(ourVoronoi.split_face(f, to_join[0], to_join[1]))

        draw_voronoi(ourVoronoi.G)

        # TODO: Eliminar lo que hay dentro de las caras D:.
        print("punching the hole")
        ourVoronoi.delete_interior(new_face,new_site)

    return ourVoronoi.G


if __name__ == "__main__":
    sites = xtractArguments(sys.argv)
    #plt.scatter(sites.T[0],sites.T[1])

    voronoi = main(sites)
    for arrow in voronoi[1]:
        plt.arrow(arrow[0],arrow[1],arrow[2],arrow[3],head_width=0.07,length_includes_head=True,shape='left')
    plt.scatter(voronoi[0].T[0],voronoi[0].T[1])
    plt.scatter(sites.T[0],sites.T[1])
    plt.show()

