import numpy as np
import sys
sys.path.append("/home/ric/Documents/Escuela/7mo_semestre/CompGeo/Tarea3/Modules/PyS4DCEL/S4DCELpp/cmake-build-debug")
from typing import List

import PyS4DCEL as pdcl

IDX_EXT_FACE = 0


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
    V = [(-1.2, -1.2), ( 1.2, -1.2), ( 1.2,  1.2), (-1.2,  1.2)]
    # En este orden en particular, la cara de afuera del marco convenientemente
    # será la del índice 0.
    E = [(0, 1), (1, 2), (2, 3), (3, 0)]
    ourVoronoi = pdcl.dcel(V, E)

    # Supondremos que el primer sitio le pertenece TODA la cara del marco.
    sites_already_visited = list()
    sites_already_visited.append(sites[0])

    # Por cada nuevo sitio, actualizaremos el diagrama de Voronoi.
    for i in range(1, len(sites)):
        new_site        = sites[i]
        faces_to_divide = dict()

        # Conforme encntremos las intersecciones usando bisectores, guardaremos
        # las caras vecinas. Esto lo repetiremos hasta que las intersecciones
        # dejen de tocar una arista que toca una cara no explorada.
        facesExplored  = list()
        facesToExplore = list()

        lines_surround_face = list()

        # Siempre comenzamos con la cara en la que cae nuestro nuevo sitio.
        facesToExplore.append(ourVoronoi.landing_face(new_site))

        while facesToExplore:
            fid = facesToExplore.pop()
            facesExplored.append(fid)
            landing_site = None

            # Buscaremos a cuál sitio le pertenecía la cara anteriormente.
            for sit in sites_already_visited:
                if ourVoronoi.are_faces_eq(ourVoronoi.landing_face(sit), fid):
                    landing_site = sit
                    break
            # TODO: Lo anterior no es muy eficiente. ¿Habrá alguna manera de guardar
            # el sitio de Voronoi para cada cara? Es necesario ir actualizando la
            # cara a la que pertenece cada sitio.
            # TODO: No toma ninguna cara cuando un vértice está justo encima de una
            # arista.

            if landing_site is None:
                continue

            # Podemos calcular el bisector y guardaremos los datos para, más
            # adelante, poder dividir la cara.
            bisector = pdcl.get_bisector(new_site, landing_site)
            bound    = ourVoronoi.get_boundry(fid)
            intersec = []
            to_split = []
            for line in bound:
                aux = pdcl.get_intersection(bisector, line)
                
                if aux is not None:
                    intersec.append(aux)
                    to_split.append(line.on_bound_id)

                    face_a, face_b = ourVoronoi.faces_touch_line(line.on_bound_id)
                    if not pdcl.is_face_inside_list(face_a, facesToExplore + facesExplored):
                        facesToExplore.append(face_a)
                    if not pdcl.is_face_inside_list(face_b, facesToExplore + facesExplored):
                        facesToExplore.append(face_b)

            faces_to_divide[fid] = (intersec[:], to_split[:])

        # Ahora haremos todas las divisiones de todas las caras.
        for f, (inter, split) in faces_to_divide.items():
            to_join = []

            for i, p in enumerate(inter):
                isAVert, vertId = ourVoronoi.is_a_vert(p[0], p[1])

                # En caso de que el vértice que intentamos crear, ya existe,
                # simplemente lo tomamos sin crear uno nuevo.
                if isAVert:
                    to_join.append(vertId)
                else:
                    to_join.append(ourVoronoi.split_edge(tuple(p), split[i]))

            lines_surround_face.append(ourVoronoi.split_face(f, to_join[0], to_join[1]))

        # TODO: Eliminar lo que hay dentro de las caras D:.
        # lines_surround_face = list(map(pdcl.line.edge_id, bisectors_of_new_face))
        # lines_surround_face = list(map(lambda x : x.edge_id, bisectors_of_new_face))
        if lines_surround_face:
            ourVoronoi.sort_lines(lines_surround_face, tuple(new_site), IDX_EXT_FACE)
        # ourVoronoi.delete_interior_sorted(lines_surround_face)

        sites_already_visited.append(new_site)

    return ourVoronoi.G


if __name__ == "__main__":
    sites = xtractArguments(sys.argv)

    main(sites)

