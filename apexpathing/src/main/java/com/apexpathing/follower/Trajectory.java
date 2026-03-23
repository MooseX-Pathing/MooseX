package com.apexpathing.follower;

/**
 * Trajectory interface for the CustomDrive base class.
 * @author Krish Joshi - 26192 Heatwaves
 */
public interface Trajectory {
    /**
     * Samples the trajectory at a given parametric time.
     * @param t that varies from [0, 1].
     * @return The trajectory sample at the given parametric time.
     */
    TrajectorySample sample(double t);

    /**
     * @return Arc length of trajectory
     */
    double length();

    /**
     * @param s Distance along the arc [0,length]
     * @return A {@link TrajectorySample} at the given arc length
     */
    default TrajectorySample duration(double s) {
        return sample(s/length());
    }
}
