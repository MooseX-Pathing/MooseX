package com.apexpathing.drivetrain.drivetrainTest;


import com.apexpathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ForwardVelocityTuner
 * Drives the robot forward at full power for duration seconds,
 * then records peak ticks/sec from drive motor encoders.
 *Works with: MecanumDrive, TankDrive, SwerveDrive
 *HOW TO USE:
 *   1. Run this opmode.
 *   2. Press A to start the drive burst.
 *   3. Read peakTicksPerSec from telemetry/dashboard.
 *   4. Plug that value into your DriveConstants as MAX_TICKS_PER_SEC (forward).
 *
 * @author Sohum Arora 22985 Paraducks
 */

@TeleOp(name = "Forward Vel Tuner", group = "ApexPathing Tuning")
public class ForwardVelocityTuner extends LinearOpMode {


    private Drivetrain drivetrain;
    public static double duration = 3;

    @Override
    public void runOpMode() {

       //TODO initialize your drivetrain here

        telemetry.addLine("Forward Velocity Tuner Ready");
        telemetry.addLine("Press A to start drive burst.");
        telemetry.update();

        waitForStart();

        double peakTicksPerSec = 0;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                ElapsedTime timer = new ElapsedTime();
                timer.reset();

                while (opModeIsActive() && timer.seconds() < duration) {

                    drivetrain.drive(0, 1, 0);

                    double currentVelocity = 0;
                    for (DcMotorEx motor : drivetrain.getMotors()) {
                        double vel = Math.abs(motor.getVelocity());
                        if (vel > currentVelocity) currentVelocity = vel;
                    }
                    if (currentVelocity > peakTicksPerSec) {
                        peakTicksPerSec = currentVelocity;
                    }

                    telemetry.addData("Elapsed (s)", timer.seconds());
                    telemetry.addData("Current Velocity (ticks/s)", currentVelocity);
                    telemetry.addData("Peak Velocity (ticks/s)", peakTicksPerSec);
                    telemetry.update();
                }

                drivetrain.drive(0, 0, 0);
            }

            telemetry.addLine("Press A to run tuner again");
            telemetry.addData("Peak Forward Velocity (ticks/second)", peakTicksPerSec);
            telemetry.update();
        }
    }
}