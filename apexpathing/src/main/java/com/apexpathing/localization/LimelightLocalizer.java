package com.apexpathing.localization;

import com.apexpathing.util.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A localizer using Limelight vision data.
 */
public class LimelightLocalizer implements Localizer {
    private Pose currentPose = new Pose(0, 0, 0);
    private Pose currentVelocity = new Pose(0, 0, 0);

    public LimelightLocalizer(HardwareMap hardwareMap) {
        // Initialization logic for Limelight would go here
    }

    @Override
    public void update() {
        // Limelight update logic:
        // 1. Get latest vision result
        // 2. Transform to field coordinates
        // 3. Update currentPose
        // For now, this is a skeleton.
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public void setPose(Pose pose) {
        this.currentPose = pose;
    }

    public Pose updateWithVision(double x, double y, double heading) {
        Pose returnPose = new Pose(x, y, heading);
        this.currentPose = returnPose;
        return returnPose;
    }
}
