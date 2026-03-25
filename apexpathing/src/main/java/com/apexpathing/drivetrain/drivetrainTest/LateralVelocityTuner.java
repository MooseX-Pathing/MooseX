package com.apexpathing.drivetrain.drivetrainTest;

import com.apexpathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * LateralVelocityTuner
 * Strafes the robot right at full power for DRIVE_DURATION seconds,
 then records peak ticks/sec from drive motor encoders.
 * Works for mec and swerve drive
 * HOW TO USE:
 *   1. Run this opmode.
 *   2. Press A to start the strafe burst.
 *   3. Read PEAK_TICKS_PER_SEC from telemetry/dashboard.
 *   4. Plug into your DriveConstants as MAX_TICKS_PER_SEC (lateral).
 * @author Sohum Arora 22985 Paraducks
 */
@TeleOp(name = "Lateral Velocity Tuner", group = "ApexPathing Tuning")
public class LateralVelocityTuner extends LinearOpMode {


    public static double duration = 3;
    public static boolean STRAFE_RIGHT  = true;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {

        //todo initialize drivetrain here

        telemetry.addLine("Lateral Velocity Tuner Ready");
        telemetry.addLine("ONLY valid for Mecanum / Swerve");
        telemetry.addLine("Press A to start");
        telemetry.update();

        waitForStart();

        double peakTicksPerSec = 0;
        double strafeDir = STRAFE_RIGHT ? 1.0 : -1.0;


        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                running = !running;
            }
            if (gamepad1.a) {
                strafeDir = STRAFE_RIGHT ? 1.0 : -1.0;
                ElapsedTime timer = new ElapsedTime();
                timer.reset();

                while (opModeIsActive() && timer.seconds() < duration) {

                    drivetrain.drive(1 * strafeDir, 0, 0);

                    double maxVel = 0;
                    for (DcMotorEx motor : drivetrain.getMotors()) {
                        double vel = Math.abs(motor.getVelocity());
                        if (vel > maxVel) maxVel = vel;
                    }
                    if (maxVel > peakTicksPerSec) peakTicksPerSec = maxVel;

                    telemetry.addData("Elapsed (s)", timer.seconds());
                    telemetry.addData("Strafe Direction", STRAFE_RIGHT ? "RIGHT" : "LEFT");
                    telemetry.addData("Current Vel (ticks/s)", maxVel);
                    telemetry.addData("Peak Vel (ticks/s)",  peakTicksPerSec);
                    telemetry.update();
                }

                drivetrain.drive(0, 0, 0);
            }

            telemetry.addLine("Press A to run tuner again");
            telemetry.addData("Peak Lateral Vel (ticks/s)", peakTicksPerSec);

            telemetry.update();
        }
    }
}