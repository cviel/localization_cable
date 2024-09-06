import numpy as np
from scipy.linalg import eig, inv


def compute_calibration_parameters(data):
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    # Create column vectors
    magnetomIndex = len(x)
    x = np.reshape(x, (magnetomIndex, 1))
    y = np.reshape(y, (magnetomIndex, 1))
    z = np.reshape(z, (magnetomIndex, 1))

    # Form the D matrix for the ellipsoid fitting
    D = np.hstack([
        x * x,
        y * y,
        z * z,
        2 * x * y,
        2 * x * z,
        2 * y * z,
        2 * x,
        2 * y,
        2 * z
    ])

    # Solve the normal system of equations
    ones = np.ones((magnetomIndex, 1))
    tempA = np.dot(D.T, D)
    tempB = np.dot(D.T, ones)
    v = np.linalg.solve(tempA, tempB).flatten()

    # Form the algebraic form of the ellipsoid
    A = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], -1]
    ], dtype=float)

    # Find the center of the ellipsoid
    center = -np.linalg.solve(A[:3, :3], v[6:9])

    # Form the corresponding translation matrix
    T = np.eye(4)
    T[3, :3] = center.T

    # Translate to the center
    R = T @ A @ T.T

    # Solve the eigenproblem
    evals, evecs = eig(R[:3, :3] / -R[3, 3])
    radii = np.sqrt(1.0 / evals.real)

    # Calculate compensation matrix
    scale = inv(np.diag(radii)) * np.min(radii)
    map1 = evecs.T
    invmap1 = evecs
    comp = invmap1 @ scale @ map1

    return center, comp


if __name__ == '__main__':
    center, comp = compute_calibration_parameters()
    # Print results
    print("Ellipsoid center:", center.flatten())
    # print("Eigenvectors (principal axes):")
    # print(evecs)
    # print("Radii of the ellipsoid:", radii)
    print("Compensation matrix:")
    print(comp)

    # Save the compensation matrix to a file
    np.savetxt('../cfg/magn_ellipsoid_transform.txt', comp, fmt='%f')
    # Save the ellipsoid center to a file
    np.savetxt('../cfg/magn_ellipsoid_center.txt', center.flatten(), fmt='%f')

    print("Saved in cfg folder")