package com.apexpathing.drivetrain.drivetrainTest;

import com.apexpathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * DecelerationTest
 *
 * Drives forward at full power for ACCELERATE_DURATION seconds,
 then cuts power and measures how far (in ticks) the bot coasts to a stop.
 * Works with: MecanumDrive, TankDrive, SwerveDrive
 * HOW TO USE:
 *   1. Run opmode, place robot against a wall or mark start position.
 *   2. Press A to begin test.
 *   3. After bot stops, read COAST_TICKS from telemetry.
 *   4. Use this to estimate kDecel or braking distance in your path follower.
 *      If coast distance is too large, enable BRAKE mode in your drivetrain.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author Simon Gidrewics 31663 RSR Cuttlefish
 */
@TeleOp(name = "Deceleration Test", group = "ApexPathing Tuning")
public class DecelerationTest extends LinearOpMode {
    public static double accelerateDuration = 2.0;   // seconds at full power
    public static double timeout = 5.0;   // max seconds to wait for stop
    public static double stopThreshold = 50.0;  // ticks/sec — "stopped"
    private Drivetrain drivetrain;
    private boolean running;

    @Override
    public void runOpMode() {

        //TODO initialize drivetrain here


        telemetry.addLine("Deceleration Test Ready");
        telemetry.addLine("Clear space ahead of robot!");
        telemetry.addLine("Press A to run.");
        telemetry.update();

        waitForStart();

        double coastTicks = 0;
        double coastTimeResult = 0;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                running = !running;
            }
            if (running) {

                // Phase 1: Accelerate
                ElapsedTime timer = new ElapsedTime();
                long startTicks = getAvgEncoderTicks();

                timer.reset();
                while (opModeIsActive() && timer.seconds() < accelerateDuration) {
                    drivetrain.drive(0, 1.0, 0);
                    telemetry.addData("Phase",   "ACCELERATING");
                    telemetry.addData("Elapsed",  timer.seconds());
                    telemetry.update();
                }

                long ticksAtCutoff = getAvgEncoderTicks();
                double velAtCutoff = getMaxVelocity();

                // Phase 2: Cut power, coast to stop
                drivetrain.drive(0, 0, 0);
                // drivetrain.breakFollowing(); // uncomment to test with brake mode

                ElapsedTime coastTimer = new ElapsedTime();
                coastTimer.reset();

                while (opModeIsActive() && coastTimer.seconds() < timeout) {
                    double currentVel = getMaxVelocity();
                    if (currentVel < stopThreshold) break;

                    telemetry.addData("Phase",       "COASTING");
                    telemetry.addData("Velocity",    "%.1f ticks/s", currentVel);
                    telemetry.update();
                }

                long ticksAtStop  = getAvgEncoderTicks();
                coastTicks = Math.abs(ticksAtStop - ticksAtCutoff);
                coastTimeResult = coastTimer.seconds();
            }

            telemetry.addData("Coast Distance (ticks)", coastTicks);
            telemetry.addData("Coast Time (s)", coastTimeResult);
            telemetry.addLine("Press A to run tuner again");
            telemetry.update();
        }
    }

    private long getAvgEncoderTicks() {
        long sum = 0;
        List<DcMotorEx> motors = drivetrain.getMotors();
        for (DcMotorEx m : motors) sum += m.getCurrentPosition();
        return sum / motors.size();
    }

    private double getMaxVelocity() {
        double max = 0;
        for (DcMotorEx motor : drivetrain.getMotors())
            max = Math.max(max, Math.abs(motor.getVelocity()));
        return max;
    }
}
