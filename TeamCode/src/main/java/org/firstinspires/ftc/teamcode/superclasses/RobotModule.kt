package org.firstinspires.ftc.teamcode.superclasses

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class RobotModule {
    lateinit var opMode: LinearOpMode

    open fun initialize() {}

    fun initialize(opMode: LinearOpMode) {
        this.opMode = opMode
        initialize()
    }

    open fun start() {}
    open fun updateAll() {}

    open val status: String
    get() = this.toString()
}