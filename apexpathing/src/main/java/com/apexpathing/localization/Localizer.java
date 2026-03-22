package com.apexpathing.localization;

import com.apexpathing.util.math.Pose;

/**
 * Interface for all localizers.
 */
public interface Localizer {
    /**
     * Updates the localizer.
     */
    void update();

    /**
     * Gets the robot's current pose.
     * @return The current pose.
     */
    Pose getPose();

    /**
     * Gets the robot's current velocity.
     * @return The current velocity.
     */
    Pose getVelocity();

    /**
     * Sets the robot's current pose.
     * @param pose The new pose.
     */
    void setPose(Pose pose);
}
