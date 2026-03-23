package com.apexpathing.follower;

import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.PoseKt;
import com.apexpathing.util.math.Vector;

/**
 * A quintic Hermite spline solver and evaluator.
 * Solves for 5th-degree polynomial coefficients given start/end poses, velocities, and accelerations.
  * @author Krish Joshi - 26192 Heatwaves
 */
public class QuinticHermiteSpline implements Trajectory {
    private final double[] xCoeffs;
    private final double[] yCoeffs;
    private final double startHeading;
    private final double endHeading;

    public QuinticHermiteSpline(
            Pose start, Vector startVel, Vector startAccel,
            Pose end, Vector endVel, Vector endAccel
    ) {
        this.xCoeffs = solve(start.x(), startVel.x(), startAccel.x(), end.x(), endVel.x(), endAccel.x());
        this.yCoeffs = solve(start.y(), startVel.y(), startAccel.y(), end.y(), endVel.y(), endAccel.y());
        this.startHeading = start.heading();
        this.endHeading = end.heading();
    }

    private double[] solve(double p0, double v0, double a0, double p1, double v1, double a1) {
        double f = p0;
        double e = v0;
        double d = a0 / 2.0;

        double targetP = p1 - p0 - v0 - a0 / 2.0;
        double targetV = v1 - v0 - a0;
        double targetA = a1 - a0;

        double a = 6 * targetP - 3 * targetV + 0.5 * targetA;
        double b = -15 * targetP + 7 * targetV - targetA;
        double c = 10 * targetP - 4 * targetV + 0.5 * targetA;

        return new double[]{a, b, c, d, e, f};
    }

    public Vector getPoint(double t) {
        return new Vector(evaluate(xCoeffs, t), evaluate(yCoeffs, t));
    }

    public Vector getVelocity(double t) {
        return new Vector(evaluateDerivative(xCoeffs, t), evaluateDerivative(yCoeffs, t));
    }

    public Vector getAcceleration(double t) {
        return new Vector(evaluateSecondDerivative(xCoeffs, t), evaluateSecondDerivative(yCoeffs, t));
    }

    private double evaluate(double[] coeffs, double t) {
        return ((((coeffs[0] * t + coeffs[1]) * t + coeffs[2]) * t + coeffs[3]) * t + coeffs[4]) * t + coeffs[5];
    }

    private double evaluateDerivative(double[] coeffs, double t) {
        return (((5 * coeffs[0] * t + 4 * coeffs[1]) * t + 3 * coeffs[2]) * t + 2 * coeffs[3]) * t + coeffs[4];
    }

    private double evaluateSecondDerivative(double[] coeffs, double t) {
        return ((20 * coeffs[0] * t + 12 * coeffs[1]) * t + 6 * coeffs[2]) * t + 2 * coeffs[3];
    }

    @Override
    public TrajectorySample sample(double t) {
        double interpolation = Math.max(0.0, Math.min(1.0, t));

        Vector p = getPoint(interpolation);
        Vector v = getVelocity(interpolation);
        Vector a = getAcceleration(interpolation);

        double heading = startHeading + PoseKt.normalize(endHeading - startHeading) * interpolation;
        double vtheta = PoseKt.normalize(endHeading - startHeading);

        return new TrajectorySample(
                new Pose(p.x(), p.y(), PoseKt.normalize(heading)),
                new Pose(v.x(), v.y(), vtheta),
                new Pose(a.x(), a.y(), 0.0)
        );
    }

    @Override
    public double length() {
        // Numerical integration for arc length
        int n = 100;
        double length = 0;
        double dt = 1.0 / n;
        for (int i = 0; i < n; i++) {
            double t = (i + 0.5) * dt;
            length += getVelocity(t).magnitude() * dt;
        }
        return length;
    }
}
