@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.avoidRingStack
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.moveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.pickSecondWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.pickupRings
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shooting
import org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor
import org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime

@Autonomous
class Auto_Ultimate : AutoOpMode() {
    override fun main() {
        conveyor.enableFullStackStopping = true
        conveyor.reverseBeforeStop = true
        avoidRingStack()
        shooting()
        moveWobble()
        pickupRings()
        if (runTime.seconds() < 23) {
            pickSecondWobble()
            moveWobble()
        }
        park()
    }
}