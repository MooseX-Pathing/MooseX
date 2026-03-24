package com.apexpathing.follower.ThreeWheelFollowers;

import com.apexpathing.drivetrain.MecanumDrive;

import com.apexpathing.localization.ThreeWheelLocalizer;
import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;

public class PTPFollowerMec {

    private final MecanumDrive drive;
    public final ThreeWheelLocalizer localizer;

    private Pose currentPose;
    private Pose targetPose;

    private double translationalKp = 0.1;
    private double headingKp = 0.5;
    private double translationalTolerance = 1.0;
    private double headingTolerance = 0.05;
    public static boolean isBusy;

    public PTPFollowerMec(MecanumDrive drive, ThreeWheelLocalizer localizer) {
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

            Vector error = new Vector(dx, dy);
            double headingError = normalizeAngle(targetPose.heading() - currentPose.heading());

            Vector rotated = error.copy();
            rotated.rotateVec(-currentPose.heading());

            double x = rotated.getXComponent() * translationalKp;
            double y = rotated.getYComponent() * translationalKp;
            double turn = headingError * headingKp;

            drive.botCentricDrive(x, y, turn);
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