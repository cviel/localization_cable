import codac as cdc
import numpy as np
import matplotlib.pyplot as plt

### Définition des contracteurs

ctc_a1a2 = cdc.CtcFunction(
    cdc.Function(
        "alpha",
        "beta",
        "mu",
        "eta",
        "a1",
        "a2",
        "(sqrt(1 + tan(mu)^2 * cos(alpha)^2) - a1 ; sqrt(1 + tan(eta)^2 * cos(beta)^2) - a2)",
    )
)

ctc_a3a4 = cdc.CtcFunction(
    cdc.Function(
        "alpha",
        "beta",
        "mu",
        "eta",
        "a3",
        "a4",
        "(sqrt(sin(mu)^2 + cos(mu)^2 / cos(alpha)^2) - a3 ; sqrt(sin(eta)^2 + cos(eta)^2 / cos(beta)^2) - a4)",
    )
)

ctc_l1 = cdc.CtcFunction(
    cdc.Function(
        "z",
        "alpha",
        "beta",
        "mu",
        "eta",
        "a1",
        "a2",
        "a3",
        "a4",
        "L",
        "l1",
        "((L*cos(beta)/a2 + z) / (cos(alpha)/a1 + cos(beta)/a2) - l1 ; (L*cos(eta)/a4 + z) / (cos(mu)/a3 + cos(eta)/a4) - l1)",
    )
)

ctc_l2 = cdc.CtcFunction(
    cdc.Function(
        "z",
        "alpha",
        "beta",
        "mu",
        "eta",
        "a1",
        "a2",
        "a3",
        "a4",
        "L",
        "l2",
        "((L*cos(alpha)/a1 - z) / (cos(alpha)/a1 + cos(beta)/a2) - l2 ; (L*cos(mu)/a3 - z) / (cos(mu)/a3 + cos(eta)/a4) - l2)",
    )
)

ctc_L = cdc.CtcFunction(cdc.Function("l1", "l2", "L", "l1 + l2 - L"))

ctc_l1l2xy = cdc.CtcFunction(
    cdc.Function(
        "a1",
        "a2",
        "a3",
        "a4",
        "l1",
        "l2",
        "l1x",
        "l1y",
        "l2x",
        "l2y",
        "(l1 / a1 - l1x ; l1 / a3 - l1y ; l2 / a2 - l2x ; l2 / a4 - l2y)",
    )
)

ctc_xy = cdc.CtcFunction(
    cdc.Function(
        "alpha",
        "beta",
        "mu",
        "eta",
        "l1x",
        "l1y",
        "l2x",
        "l2y",
        "x",
        "y",
        "(l1x * sin(alpha) + l2x * sin(beta) - x ; l1y * sin(mu) + l2y * sin(eta) - y)",
    )
)

ctc_m_gliss = cdc.CtcFunction(
    cdc.Function(
        "alpha",
        "mu",
        "l1x",
        "l1y",
        "x_masse",
        "y_masse",
        "z_masse",
        "(l1x * sin(alpha) - x_masse ; l1y * sin(mu) - y_masse ; l1x * cos(alpha) - z_masse ; l1y * cos(mu) - z_masse)",
    )
)

ctc_thetas = cdc.CtcFunction(
    cdc.Function(
        "x",
        "y",
        "z",
        "thetax",
        "thetay",
        "(atan2(z, x) - thetax ; atan2(z, y) - thetay)",
    )
)

ctc_ellipse = cdc.CtcFunction(
    cdc.Function(
        "x_masse",
        "y_masse",
        "z_masse",
        "x",
        "y",
        "z",
        "L",
        "thetax",
        "thetay",
        "(4*((x_masse-x/2)*cos(thetax) + (z_masse-z/2)*sin(thetax))^2 / L^2 + 4*((x_masse-x/2)*sin(thetax) - (z_masse-z/2)*cos(thetax))^2 / (L^2 - (x^2+z^2)) - 1 ;\
        4*((y_masse-y/2)*cos(thetay) + (z_masse-z/2)*sin(thetay))^2 / L^2 + 4*((y_masse-y/2)*sin(thetay) - (z_masse-z/2)*cos(thetay))^2 / (L^2 - (y^2+z^2)) - 1)",
    )
)

# ctc_d = cdc.CtcFunction(cdc.Function("p[3]", "d", "sqrt(p[0]^2 + p[1]^2 + p[2]^2) - d"))


def main():
    pass

if __name__ == "__main__":
    main()