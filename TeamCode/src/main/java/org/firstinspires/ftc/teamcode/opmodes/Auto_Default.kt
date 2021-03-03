@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.avoidRingStack
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.moveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.pickupRings
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shootHighGoal
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shooting
import org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor

@Autonomous
class Auto_Default : AutoOpMode() {
    override fun main() {
        conveyor.enableFullStackStopping = true
        conveyor.reverseBeforeStop = true
        avoidRingStack()
        shootHighGoal()
        moveWobble()
        pickupRings()
        park()
    }
}