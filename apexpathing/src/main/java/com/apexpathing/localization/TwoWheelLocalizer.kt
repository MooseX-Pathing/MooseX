package com.apexpathing.localization

import com.apexpathing.util.math.Pose
import com.apexpathing.util.math.Vector
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.cos
import kotlin.math.sin

/**
 * @author Topher Fontana on 3-21-26
 */
class TwoWheelLocalizer(val wheelRadius:Double = 48.0, val hubOrientation: RevHubOrientationOnRobot, private var leftPodName: String, private var rightPodName: String, private var leftPodResolution: Resolution, private var rightPodResolution: Resolution, private var hubName:String = "imu"): LocalizerBase() {
    private lateinit var odometryPods: Array<OdometryPod>
    private var lastTimeStamp: Double = 0.0
    private val timer: ElapsedTime = ElapsedTime()
    private lateinit var imu: IMU

    override fun initLocalizer(hardwareMap: HardwareMap) {
        imu = hardwareMap.get(IMU::class.java, hubName)
        imu.initialize(IMU.Parameters(hubOrientation))

        odometryPods = arrayOf(
            OdometryPod(hardwareMap.get(DcMotor::class.java, leftPodName) as DcMotorEx, leftPodResolution),
            OdometryPod(hardwareMap.get(DcMotor::class.java, rightPodName) as DcMotorEx, rightPodResolution)
        )
    }

    override fun update() {
        val timestamp: Double = timer.milliseconds()
        val dt = timestamp - lastTimeStamp
        lastTimeStamp = timestamp

        val dLW = odometryPods[0].getAngleTravelled() * wheelRadius
        val dRW = odometryPods[1].getAngleTravelled() * wheelRadius

        val dForward = (dLW + dRW) / 2.0
        val currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        val dHeading = currentHeading - currentPosition.heading
        val thetaMid = currentPosition.heading + dHeading / 2.0

        lastPosition = currentPosition
        currentPosition = Pose(currentPosition.x + dForward * cos(thetaMid), currentPosition.y + dForward * sin(thetaMid), currentHeading)

        lastVelocity = currentVelocity
        currentVelocity = Vector((currentPosition.x - lastPosition.x) / dt, (currentPosition.y - lastPosition.y) / dt)

        currentAcceleration = (currentVelocity - lastVelocity) / dt

        for(o in odometryPods) {
            o.update()
        }
    }

    override fun setPose(pose: Pose) {
        currentPosition = pose
        lastPosition = pose
    }

    fun resetYaw() {
        imu.resetYaw()
    }
}
