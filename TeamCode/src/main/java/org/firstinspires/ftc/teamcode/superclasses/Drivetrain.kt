package org.firstinspires.ftc.teamcode.superclasses

import org.firstinspires.ftc.teamcode.math.Vector3D

interface Drivetrain {

    var targetVelocity: Vector3D

    val maxVelocity: Vector3D
}