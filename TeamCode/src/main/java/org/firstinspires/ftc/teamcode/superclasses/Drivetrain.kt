package org.firstinspires.ftc.teamcode.superclasses

import org.firstinspires.ftc.teamcode.math.Vector3D

interface Drivetrain {
    fun setRobotVelocity(frontways: Double, sideways: Double, turn: Double)
    fun setRobotVelocity(move: Vector3D) {
        setRobotVelocity(move.y, move.x, move.z)
    }

    val maxVelocity: Vector3D
}