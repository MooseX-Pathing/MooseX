package com.apexpathing.localization


import com.apexpathing.util.math.Vector
import com.apexpathing.util.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 *@author Topher Fontana (@moosecoding)
 *@date 3/19/26
 */



abstract class LocalizerBase() : Localizer {
    // Add the @Pathing annotation stuff here

    // Stores the last position so that way we can calculate velocity
    var lastPosition: Pose = Pose(0.0,0.0,0.0)
    // Stores the last velocity so that way we can calculate acceleration
    var lastVelocity: Vector = Vector(0.0,0.0)
    var currentPosition: Pose = Pose(0.0,0.0,0.0)
    var currentVelocity: Vector = Vector(0.0,0.0)
    var currentAcceleration: Vector = Vector(0.0,0.0)

    override fun getPose(): Pose = currentPosition
    override fun getVelocity(): Pose = Pose(currentVelocity.x, currentVelocity.y, 0.0)



    /**
     * This initializes the localizer's hardware map or whatever else needed
     */
    abstract fun initLocalizer(hardwareMap: HardwareMap)


    /**
     * This is the update class called when your localizer is attached using the @Pathing annotation
     */
    abstract override fun update()

    /**
     *@param A [Pose] to set the location of the localizer to
     */
    abstract override fun setPose(pose: Pose)
}

