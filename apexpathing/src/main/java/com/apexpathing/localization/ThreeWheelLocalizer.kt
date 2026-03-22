package com.apexpathing.localization

import com.apexpathing.util.math.Pose
import com.apexpathing.util.math.Vector
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos
import kotlin.math.sin
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * @author Topher Fontana on 3-21-26
 */
class ThreeWheelLocalizer(val wheelRadius:Double = 48.0, private var leftPodName: String, private var rightPodName: String, private var strafePodName: String, private var leftPodResolution: Resolution, private var rightPodResolution: Resolution, private var strafePodResolution: Resolution, private var width: Double = 18.0, private var strafeOffset: Double = 0.0): LocalizerBase() {
    private lateinit var odometryPods: Array<OdometryPod>

    private var lastTimeStamp: Double = 0.0

    private val timer: ElapsedTime = ElapsedTime()

    override fun initLocalizer(hardwareMap: HardwareMap) {
        odometryPods = arrayOf(
            OdometryPod( hardwareMap.get(DcMotor::class.java, leftPodName) as DcMotorEx, leftPodResolution),
            OdometryPod( hardwareMap.get(DcMotor::class.java, rightPodName) as DcMotorEx, rightPodResolution),
            OdometryPod( hardwareMap.get(DcMotor::class.java, strafePodName) as DcMotorEx, strafePodResolution)
        )
    }

    override fun update() {
        val timestamp: Double = timer.milliseconds()
        val dt = timestamp - lastTimeStamp
        lastTimeStamp = timestamp

        val dLW = odometryPods[0].getAngleTravelled() * wheelRadius
        val dRW = odometryPods[1].getAngleTravelled() * wheelRadius
        val dSW = odometryPods[2].getAngleTravelled() * wheelRadius

        val dHeading = (dLW - dRW) / width
        val dForward = (dLW + dRW) / 2.0
        val dStrafe = dSW - strafeOffset * dHeading

        val thetaMid = currentPosition.heading + dHeading / 2.0

        lastPosition = currentPosition
        currentPosition = Pose(currentPosition.x + dForward * cos(thetaMid) - dStrafe * sin(thetaMid), currentPosition.y + dForward * sin(thetaMid) + dStrafe * cos(thetaMid), currentPosition.heading + dHeading)

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
}
