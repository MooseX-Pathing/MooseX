package com.apexpathing.follower;

import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;

/**
 * A trajectory implementation that wraps multiple QuinticHermiteSpline segments
 * and maps elapsed time to the spline parameter t.
 * @author Krish Joshi - 26192 Heatwaves
 */
public class TimeTrajectory implements Trajectory {
    private final QuinticHermiteSpline[] splines;
    private final double duration;
    private final double[] segmentStartTimes;
    private final double[] segmentDurations;
    private final double[] headings;

    /**
     * @param splines Array of splines representing the path segments.
     * @param segmentDurations Duration of each segment in seconds.
     * @param headings Target headings at the start of each segment (and one for the end).
     */
    public TimeTrajectory(QuinticHermiteSpline[] splines, double[] segmentDurations, double[] headings) {
        this.splines = splines;
        this.segmentDurations = segmentDurations;
        this.headings = headings;

        this.segmentStartTimes = new double[splines.length];
        double totalDuration = 0;
        for (int i = 0; i < splines.length; i++) {
            segmentStartTimes[i] = totalDuration;
            totalDuration += segmentDurations[i];
        }
        this.duration = totalDuration;
    }

    @Override
    public TrajectorySample sample(double time) {
        if (time <= 0) {
            Vector p = splines[0].getPoint(0);
            Vector v = splines[0].getVelocity(0).div(segmentDurations[0]);
            Vector a = splines[0].getAcceleration(0).div(segmentDurations[0] * segmentDurations[0]);
            double vtheta = normalizeAngle(headings[1] - headings[0]) / segmentDurations[0];
            return new TrajectorySample(new Pose(p.x(), p.y(), headings[0]), new Pose(v.x(), v.y(), vtheta), new Pose(a.x(), a.y(), 0));
        }

        if (time >= duration) {
            int lastIdx = splines.length - 1;
            Vector p = splines[lastIdx].getPoint(1);
            Vector v = splines[lastIdx].getVelocity(1).div(segmentDurations[lastIdx]);
            Vector a = splines[lastIdx].getAcceleration(1).div(segmentDurations[lastIdx] * segmentDurations[lastIdx]);
            double vtheta = normalizeAngle(headings[headings.length - 1] - headings[headings.length - 2]) / segmentDurations[lastIdx];
            return new TrajectorySample(new Pose(p.x(), p.y(), headings[headings.length - 1]), new Pose(v.x(), v.y(), vtheta), new Pose(a.x(), a.y(), 0));
        }

        int segmentIdx = splines.length - 1;
        for (int i = 0; i < splines.length - 1; i++) {
            if (time < segmentStartTimes[i + 1]) {
                segmentIdx = i;
                break;
            }
        }

        double localTime = time - segmentStartTimes[segmentIdx];
        double t = localTime / segmentDurations[segmentIdx];

        Vector p = splines[segmentIdx].getPoint(t);
        Vector v = splines[segmentIdx].getVelocity(t).div(segmentDurations[segmentIdx]);
        Vector a = splines[segmentIdx].getAcceleration(t).div(segmentDurations[segmentIdx] * segmentDurations[segmentIdx]);

        // Linear interpolation for heading
        double startHeading = headings[segmentIdx];
        double endHeading = headings[segmentIdx + 1];
        double heading = startHeading + normalizeAngle(endHeading - startHeading) * t;
        double vtheta = normalizeAngle(endHeading - startHeading) / segmentDurations[segmentIdx];

        return new TrajectorySample(new Pose(p.x(), p.y(), heading), new Pose(v.x(), v.y(), vtheta), new Pose(a.x(), a.y(), 0));
    }

    @Override
    public TrajectorySample parametricSample(Double t) {
        return sample((Math.max(0.0, Math.min(1.0, t))) * duration);
    }

    @Override
    public double duration() {
        return duration;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
