package org.firstinspires.ftc.teamcode.apexPathing.examples;

import static org.firstinspires.ftc.teamcode.apexPathing.Constants.dtConstants;
import static org.firstinspires.ftc.teamcode.apexPathing.Constants.localizer;

import com.apexpathing.drivetrain.MecanumDrive;
import com.apexpathing.follower.Follower;
import com.apexpathing.follower.PTPFollower;
import com.apexpathing.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.apexPathing.Constants;

/**
 * @author Atharv Gurnani - 13085 Bionic Dutch
 */
public class TeleOpDrive extends OpMode {
    public MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(
                hardwareMap,
                dtConstants,
                localizer
        );

        Constants.init();
    }

    @Override
    public void loop() {
        telemetry.update();

        Pose currentPose = localizer.getPose();
        drive.botCentricDrive(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        telemetry.addData("Bot X", currentPose.x());
        telemetry.addData("Bot Y", currentPose.y());
        telemetry.addData("Bot Heading", currentPose.heading());
    }
}
