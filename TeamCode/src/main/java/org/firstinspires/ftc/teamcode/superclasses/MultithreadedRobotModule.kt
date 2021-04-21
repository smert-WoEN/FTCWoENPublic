package org.firstinspires.ftc.teamcode.superclasses

abstract class MultithreadedRobotModule : RobotModule() {
    override fun updateAll() {
        updateControlHub()
        updateExpansionHub()
        updateOther()
    }
    open fun updateControlHub() {}
    open fun updateExpansionHub() {}
    open fun updateOther() {}
}