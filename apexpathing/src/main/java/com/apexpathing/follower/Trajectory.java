package com.apexpathing.follower;

/**
 * Trajectory interface for the CustomDrive base class.
 * @author Krish Joshi - 26192 Heatwaves
 */
public interface Trajectory {
    /**
     * Samples the trajectory at a given time.
     * @param time The time in seconds.
     * @return The trajectory sample at the given time.
     */
    TrajectorySample sample(double time);

    /**
     * Gets the total duration of the trajectory.
     * @return The duration in seconds.
     */
    double duration();

    /**
     * Samples the trajectory parametrically by distance.
     * @param t a Double between 0,1 along the trajectory.
     * @return The trajectory sample at the given distance.
     */
    TrajectorySample parametricSample(Double t);
}
