package org.firstinspires.ftc.teamcode.apexPathing.tuners

import com.apexpathing.drivetrain.MecanumDrive
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.apexPathing.Constants.dtConstants
import org.firstinspires.ftc.teamcode.apexPathing.Constants.localizer

/**
 * @author Atharv Gurnani - 13085 Bionic Dutch
 */
class ForwardVelocityTuner: OpMode() {
    private val dist = 40.0;
    private lateinit var drive : MecanumDrive;
    override fun init() {
        drive = MecanumDrive(hardwareMap, localizer, telemetry, dtConstants,
            "lf", "rf", "lb", "rb",
                    )
        drive.initDriveTrain()
    }

    override fun loop() {
        if (localizer.currentPosition.x + localizer.currentPosition.y <= dist) {
            drive.botCentricDrive(0.0, 1.0, 0.0)
        }
        else {
            drive.botCentricDrive(0.0,0.0,0.0);
        }
    }

}