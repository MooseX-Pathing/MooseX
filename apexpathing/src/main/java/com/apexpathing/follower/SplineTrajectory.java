package com.apexpathing.follower;

/**
 * A trajectory implementation that wraps multiple QuinticHermiteSpline segments
 * and maps the parameter t [0, 1] across the entire chain.
 * @author Krish Joshi - 26192 Heatwaves
 */
public class SplineTrajectory implements Trajectory {
    private final QuinticHermiteSpline[] splines;

    /**
     * @param splines Array of splines representing the path segments.
     */
    public SplineTrajectory(QuinticHermiteSpline[] splines) {
        this.splines = splines;
    }

    @Override
    public TrajectorySample sample(double t) {
        if (splines.length == 0) return null;
        
        double interpolation = Math.max(0.0, Math.min(1.0, t));
        
        // Map t [0, 1] to segment index and local t [0, 1]
        // Each segment gets an equal portion of the t-space
        double scaledT = interpolation * splines.length;
        int segmentIdx = (int) scaledT;
        
        if (segmentIdx >= splines.length) {
            segmentIdx = splines.length - 1;
        }
        
        double localT = scaledT - segmentIdx;
        
        return splines[segmentIdx].sample(localT);
    }

    @Override
    public double length() {
        double total = 0;
        for (QuinticHermiteSpline s : splines) total += s.length();
        return total;
    }
}
