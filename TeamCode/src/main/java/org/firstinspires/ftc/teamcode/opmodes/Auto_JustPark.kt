@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park

@Autonomous
class Auto_JustPark : AutoOpMode() {
    override fun main() {
        park()
    }
}