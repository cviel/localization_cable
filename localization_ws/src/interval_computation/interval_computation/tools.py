from os import listdir
from os.path import isfile, join
from collections import deque
import numpy as np
import codac as cdc
try:
    from vibes import vibes
    vibes_available = True
except ImportError:
    vibes_available = False


def Interval_filter_nan(x):
    return cdc.Interval() if np.isnan(x) else cdc.Interval(x)


def import_values(dir_name):
    dict = {}
    fichiers = [f for f in listdir(dir_name) if isfile(join(dir_name, f))]

    for f in fichiers:
        with open(join(dir_name, f), "r") as fich:
            filename = f[:-4] # remove ".txt"
            dict[filename] = fich.read()
            dict[filename] = dict[filename].split("\n")
            try:
                dict[filename] = np.array(dict[filename][:-1])
                dict[filename] = dict[filename].astype(float)
            except:
                dict[filename] = np.array(list(map(lambda x: x.split(), dict[filename])))
                dict[filename] = np.array(dict[filename])
                dict[filename] = dict[filename].astype(float)
    
    return dict


def max_diam(X):
    return max([x.diam() for x in X if not x.is_unbounded()])

def bisect(X, i_allowed):
    i_res = i_allowed[0] # S'assurer que ça n'a pas un diamètre infini
    for j in i_allowed:
        if X[j].diam() > X[i_res].diam() and not X[j].is_unbounded():
            i_res = j
            
    return X.bisect(i_res, 0.4)

def SIVIA_cn(X0, solve_func, epsilon, i_allowed = None, draw_boxes=False):
    if i_allowed is None:
        i_allowed = range(len(X0))
        
    stack = deque([cdc.IntervalVector(X0)])
    res_y, res_out = [], []
    lf = cdc.LargestFirst(epsilon/2.0)
    k = 0

    while len(stack) > 0:
        k += 1
        X = stack.popleft()
        X0 = cdc.IntervalVector(X)

        solve_func(X)

        res_out += X0.diff(X)

        if (X.is_empty()):
            continue
        if(max_diam(X) < epsilon or k >= 10000):
            if draw_boxes is True:
                vibes.drawBox(
                    X[0].lb(), X[0].ub(), X[1].lb(), X[1].ub(),
                    "k[cyan]")
            res_y.append(X)
        elif(X.is_empty() is False):
            (X1, X2) = bisect(X, i_allowed)
            stack.append(X1)
            stack.append(X2)
    
    if k >= 10000:
        print(f"Number of contraction {k} / number of boxes {len(res_out) + len(res_y)}")

    return (res_out, res_y)


def draw_lines_cases(p, p_m):
    # Useless cases
    if p_m[0].lb() < p[0].lb() and p_m[0].ub() > p[0].ub() and (p_m[2].lb() < p[2].lb() and p_m[2].ub() > p[2].ub() or p_m[2].lb() > p[2].lb() and p_m[2].ub() < p[2].ub()) or p[0].lb() < p_m[0].lb() and p[0].ub() > p_m[0].ub() and (p[2].lb() < p_m[2].lb() and p[2].ub() > p_m[2].ub() or p[2].lb() > p_m[2].lb() and p[2].ub() < p_m[2].ub()):
        return "useless"

    # Lines of the upper vertices
    elif p_m[0].lb() < p[0].lb() and p_m[0].ub() > p[0].ub() and p_m[2].ub() > p[2].ub() or p[0].lb() < p_m[0].lb() and p[0].ub() > p_m[0].ub() and p[2].ub() > p_m[2].ub():
        return "upper"

    # Lines of the lower vertices
    elif p_m[0].lb() < p[0].lb() and p_m[0].ub() > p[0].ub() and p_m[2].lb() < p[2].lb() or p[0].lb() < p_m[0].lb() and p[0].ub() > p_m[0].ub() and p[2].lb() < p_m[2].lb():
        return "lower"

    # Lines of the left vertices
    elif p_m[0].ub() > p[0].ub() and p_m[2].lb() < p[2].lb() and p_m[2].ub() > p[2].ub() or p[0].ub() > p_m[0].ub() and p[2].lb() < p_m[2].lb() and p[2].ub() > p_m[2].ub():
        return "left"

    # Lines of the right vertices
    elif p_m[0].lb() < p[0].lb() and p_m[2].lb() < p[2].lb() and p_m[2].ub() > p[2].ub() or p[0].lb() < p_m[0].lb() and p[2].lb() < p_m[2].lb() and p[2].ub() > p_m[2].ub():
        return "right"
    
    # Lines of the first diagonal vertices
    elif p_m[0].lb() > p[0].lb() and p_m[2].lb() > p[2].lb() or p[0].lb() > p_m[0].lb() and p[2].lb() > p_m[2].lb():
        return "first_diagonal"

    else:
        return "anti_diagonal"

def append_data_file(filename, data):
    with open(filename, "a") as f:
        if type(data) == cdc.core.Interval:
            f.write(f"{data.lb()} {data.mid()} {data.ub()}\n")
        else:
            f.write(f"{data}\n")

def z_rot(p, phi=0):
    return [p[0] * np.cos(phi) - p[1] * np.sin(phi), p[0] * np.sin(phi) + p[1] * np.cos(phi), p[2]]

def draw_cube_mtpltlb(ax, p, color, alpha=1, phi=0):
    # Draw the cube
    vertices = [
        [p[0].lb(), p[1].lb(), p[2].lb()],
        [p[0].ub(), p[1].lb(), p[2].lb()],
        [p[0].ub(), p[1].ub(), p[2].lb()],
        [p[0].lb(), p[1].ub(), p[2].lb()],
        [p[0].lb(), p[1].lb(), p[2].ub()],
        [p[0].ub(), p[1].lb(), p[2].ub()],
        [p[0].ub(), p[1].ub(), p[2].ub()],
        [p[0].lb(), p[1].ub(), p[2].ub()]
    ]

    vertices = [z_rot(v, phi) for v in vertices]

    edges = [
        [vertices[0], vertices[1], vertices[2], vertices[3], vertices[0]],
        [vertices[4], vertices[5], vertices[6], vertices[7], vertices[4]],
        [vertices[0], vertices[4]],
        [vertices[1], vertices[5]],
        [vertices[2], vertices[6]],
        [vertices[3], vertices[7]]
    ]

    for edge in edges:
        x, y, z = zip(*edge)
        ax.plot(x, y, z, color=color, alpha=alpha)

def diff_list(X0, res_in):
    stack0 = deque([X0])
    stack1 = deque()

    for box in res_in:
        if len(stack0) > 0:
            while len(stack0) > 0:
                b = stack0.popleft()
                stack1 += b.diff(box)
        else:
            while len(stack1) > 0:
                b = stack1.popleft()
                stack0 += b.diff(box)
    
    return list(stack0) if len(stack0) > 0 else list(stack1)

def volume_diff_list(X0, res_in):
    return sum(map(lambda X: X.volume(), diff_list(X0, res_in)))

def union_list(l):
    res = l[0]
    for x in l[1:]:
        res = res | x
    return res


if __name__ == "__main__":
    print(volume_diff_list(cdc.IntervalVector(3, [0, 1]), [cdc.IntervalVector(3, [0.5, 1]), cdc.IntervalVector(3, [0, 0.75]), cdc.IntervalVector(3, [0.5, 0.75]), cdc.IntervalVector([[0, 1], [0.75, 1], [0.75, 1]])]))