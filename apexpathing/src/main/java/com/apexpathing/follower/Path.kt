package com.apexpathing.follower

import com.apexpathing.util.math.Pose

class Path(private val trajectories: List<Trajectory>, private val threshold: Double) : Trajectory {
    fun isFinished(currentPose: Pose): Boolean {
        return currentPose.nearPose(this.trajectories.last().parametricSample(1.0).pose, threshold)
    }

    private val endTimes: DoubleArray = run {
        var acc = 0.0
        DoubleArray(trajectories.size) { i -> acc += trajectories[i].duration(); acc }
    }
    override fun duration(): Double = endTimes.last()

    override fun sample(time: Double): TrajectorySample {
        val clamped = time.coerceIn(0.0, duration())

        var lo = 0; var hi = trajectories.lastIndex
        while (lo < hi) {
            val mid = (lo + hi) / 2
            if (endTimes[mid] < clamped) lo = mid + 1 else hi = mid
        }

        val segStart = if (lo == 0) 0.0 else endTimes[lo - 1]
        return trajectories[lo].sample(clamped - segStart)
    }

    override fun parametricSample(t: Double): TrajectorySample {
        return sample(t.coerceIn(0.0, 1.0) * duration())
    }
}