package com.apexpathing.follower

import com.apexpathing.util.math.Pose
import com.apexpathing.util.math.Units.normalizeAngle
import kotlin.math.PI

class Line(
    private val start: Pose,
    private val end: Pose,
) : Trajectory {

    override fun sample(t: Double): TrajectorySample {
        val i = t.coerceIn(0.0, 1.0)
        
        val x = start.x + (end.x - start.x) * i
        val y = start.y + (end.y - start.y) * i
        
        val startHeading = start.heading
        val endHeading = end.heading
        val heading = startHeading + normalizeAngle(endHeading - startHeading) * i
        
        // Derivatives with respect to t
        val dx = end.x - start.x
        val dy = end.y - start.y
        val dtheta = normalizeAngle(endHeading - startHeading)
        
        return TrajectorySample(
            Pose(x, y, normalizeAngle(heading)),
            Pose(dx, dy, dtheta),
            Pose(0.0, 0.0, 0.0)
        )
    }

    override fun length(): Double {
        return start.distanceTo(end)
    }
}