package com.apexpathing.follower

import com.apexpathing.util.math.Pose

abstract class Follower {
    abstract var currentPath: Path
    private var currentT = 0.0

    open var isFinished = false

    abstract fun setTarget(target: Pose)

    abstract fun update(currentPose: Pose)
}