package com.apexpathing.follower.ThreeWheelFollowers;

import com.apexpathing.drivetrain.TankDrive;
import com.apexpathing.localization.ThreeWheelLocalizer;
import com.apexpathing.util.math.Pose;

public class PTPFollowerTank {

    private final TankDrive drive;
    public final ThreeWheelLocalizer localizer;

    private Pose currentPose;
    private Pose targetPose;

    private double translationalKp = 0.1;
    private double headingKp = 0.5;
    private double translationalTolerance = 1.0;
    private double headingTolerance = 0.05;
    public static boolean isBusy;

    public PTPFollowerTank(TankDrive drive, ThreeWheelLocalizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }
    public void setTranslationalKp(double kp) { this.translationalKp = kp; }
    public void setHeadingKp(double kp) { this.headingKp = kp; }
    public void setTranslationalTolerance(double t) { this.translationalTolerance = t; }
    public void setHeadingTolerance(double t) { this.headingTolerance = t; }

    public void update(Pose target) {
        localizer.update();
        currentPose = localizer.getCurrentPosition();
        targetPose = target;

        if (currentPose != null && targetPose != null) {
            isBusy = true;

            double dx = targetPose.x() - currentPose.x();
            double dy = targetPose.y() - currentPose.y();

            double angleToTarget = Math.atan2(dy, dx);
            double headingError = normalizeAngle(angleToTarget - currentPose.heading());

            double dist = Math.hypot(dx, dy);
            double forward = dist * translationalKp;
            double turn = headingError * headingKp;

            drive.drive(0, forward, turn);
        }
    }

    public boolean isAtTarget() {
        if (currentPose == null || targetPose == null) return false;

        double dx = targetPose.x() - currentPose.x();
        double dy = targetPose.y() - currentPose.y();
        double dist = Math.hypot(dx, dy);
        double headingError = Math.abs(normalizeAngle(targetPose.heading() - currentPose.heading()));

        if (dist < translationalTolerance && headingError < headingTolerance) {
            isBusy = false;
            return true;
        }
        return false;
    }

    public Pose getPose() { return currentPose; }
    public boolean isBusy() { return isBusy; }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
