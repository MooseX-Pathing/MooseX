package org.firstinspires.ftc.teamcode.movement;

import static org.junit.Assert.*;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.movement.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.movement.pathing.BezierCurve;
import java.util.Arrays;

public class MovementTest {
    @Test
    public void testVectorMath() {
        Vector2d v1 = new Vector2d(1, 2);
        Vector2d v2 = new Vector2d(3, 4);

        Vector2d v3 = v1.plus(v2);
        assertEquals(4, v3.x, 1e-9);
        assertEquals(6, v3.y, 1e-9);

        Vector2d v4 = v1.rotate(Math.PI / 2);
        assertEquals(-2, v4.x, 1e-9);
        assertEquals(1, v4.y, 1e-9);
    }

    @Test
    public void testBezierCurve() {
        Vector2d p0 = new Vector2d(0, 0);
        Vector2d p1 = new Vector2d(1, 0);
        Vector2d p2 = new Vector2d(1, 1);
        BezierCurve curve = new BezierCurve(Arrays.asList(p0, p1, p2));

        Vector2d start = curve.getPoint(0);
        assertEquals(0, start.x, 1e-9);
        assertEquals(0, start.y, 1e-9);

        Vector2d end = curve.getPoint(1);
        assertEquals(1, end.x, 1e-9);
        assertEquals(1, end.y, 1e-9);

        // Derivative at t=0 should be n*(p1-p0) = 2*(1,0) = (2,0)
        Vector2d d0 = curve.getDerivative(0);
        assertEquals(2, d0.x, 1e-9);
        assertEquals(0, d0.y, 1e-9);

        // Derivative at t=1 should be n*(p2-p1) = 2*(0,1) = (0,2)
        Vector2d d1 = curve.getDerivative(1);
        assertEquals(0, d1.x, 1e-9);
        assertEquals(2, d1.y, 1e-9);
    }
}
