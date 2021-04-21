package org.firstinspires.ftc.teamcode.superclasses

import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D

class MotionTask : Pose2D {
    var actionOnCompletion = Runnable {}

    constructor(x: Double = Double.NaN, y: Double = Double.NaN, heading: Double = Double.NaN, actionOnCompletion: Runnable = Runnable {}) : super(x, y, heading) {
        this.actionOnCompletion = actionOnCompletion
    }

    constructor(vector: Vector2D, heading: Double = Double.NaN, actionOnCompletion: Runnable = Runnable {}) : super(vector.x, vector.y, heading) {
        this.actionOnCompletion = actionOnCompletion
    }

    constructor(pose: Pose2D, actionOnCompletion: Runnable = Runnable {}) : super(pose.x, pose.y, pose.heading) {
        this.actionOnCompletion = actionOnCompletion
    }


    override fun clone(): MotionTask {
        return MotionTask(x, y, heading, actionOnCompletion)
    }
}