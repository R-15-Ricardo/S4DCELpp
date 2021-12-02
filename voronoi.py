import numpy as np
import sys
from typing import List

import build.PyS4DCEL as pdcl


def xtractArguments(args : List[str]):
    if len(args) < 2:
        print(f"\nUsa el programa de la siguiente manera:\n\t./{args[0]} <sites.txt>")
        sys.exit(1)

    sites = np.loadtxt(args[1])
    return sites



def main(sites: np.ndarray):
    pdcl.dcel



if __name__ == "__main__":
    sites = xtractArguments(sys.argv)

    main(sites)

