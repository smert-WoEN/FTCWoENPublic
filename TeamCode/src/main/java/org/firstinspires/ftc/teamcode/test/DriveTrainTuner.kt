package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.initRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import kotlin.math.sin

@TeleOp
class DriveTrainTuner: LinearOpMode() {
    override fun runOpMode() {
        val sineWaveTimer = ElapsedTime()
        val sineWaveResetter = SinglePressButton{gamepad1.dpad_up}
        initRobot(this)
        startRobot()
        while (opModeIsActive()) {
            if(sineWaveResetter.get()) sineWaveTimer.reset()
            movement.humanSetVelocity(gamepad1.left_stick_x.toDouble(),if(gamepad1.dpad_up) sin(sineWaveTimer.seconds() * Math.PI / 3.0)*0.5 else -gamepad1.left_stick_y.toDouble(),
                                                gamepad1.right_trigger.toDouble() -  gamepad1.left_trigger.toDouble())
        }
    }
}