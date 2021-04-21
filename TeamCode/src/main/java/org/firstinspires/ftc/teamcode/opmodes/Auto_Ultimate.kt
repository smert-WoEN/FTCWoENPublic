@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.avoidRingStack
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.moveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.pickupRings
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shooting

@Autonomous
class Auto_Ultimate : AutoOpMode() {
    override fun main() {
        avoidRingStack()
        shooting()
        moveWobble()
        pickupRings(true)
        moveWobble()
        /* if (runTime.seconds() < 23) {
             pickSecondWobble()
             moveWobble()
         }*/
        park()
    }
}