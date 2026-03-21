package org.firstinspires.ftc.teamcode.movement.pathing;

import org.firstinspires.ftc.teamcode.movement.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * A Bezier curve class implementing parametric P(t) for t in [0, 1].
 */
public class BezierCurve {
    private final List<Vector2d> controlPoints;

    public BezierCurve(List<Vector2d> controlPoints) {
        if (controlPoints.size() < 2) {
            throw new IllegalArgumentException("At least two control points are required.");
        }
        this.controlPoints = new ArrayList<>(controlPoints);
    }

    /**
     * Calculates the point on the curve at parameter t.
     */
    public Vector2d getPoint(double t) {
        return deCasteljau(controlPoints, t);
    }

    /**
     * Calculates the derivative (velocity vector) at parameter t.
     */
    public Vector2d getDerivative(double t) {
        int n = controlPoints.size() - 1;
        if (n == 0) return new Vector2d(0, 0);

        List<Vector2d> derivativePoints = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            derivativePoints.add(controlPoints.get(i + 1).minus(controlPoints.get(i)).times(n));
        }

        return deCasteljau(derivativePoints, t);
    }

    /**
     * Helper for de Casteljau's algorithm.
     */
    private Vector2d deCasteljau(List<Vector2d> points, double t) {
        if (points.size() == 1) return points.get(0);

        List<Vector2d> nextPoints = new ArrayList<>();
        for (int i = 0; i < points.size() - 1; i++) {
            nextPoints.add(points.get(i).times(1 - t).plus(points.get(i + 1).times(t)));
        }

        return deCasteljau(nextPoints, t);
    }

    public List<Vector2d> getControlPoints() {
        return controlPoints;
    }
}
