package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor
import org.firstinspires.ftc.teamcode.robot.WoENrobot.initRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.robot.Shooter

@TeleOp
class TeleOp_Dual : LinearOpMode() {
    override fun runOpMode() {
        initRobot(this)
        startRobot()
        /* Wobble */
        val threeRingPresser = SinglePressButton { gamepad1.right_stick_button }
        val grabWobbleSwitch = ButtonSwitch { gamepad2.right_bumper }
        /* Conveyor */
        val conveyorOnOffSwitch = ButtonSwitch { gamepad2.left_trigger > 0.5 }
        /* Shooter */
        val shooterSpeedSwitch = ButtonSwitch { gamepad2.y }
        val shooterOnOffSwitch = ButtonSwitch { gamepad2.a }
        while (opModeIsActive()) {
            /* Wobble */
            wobbleManipulator.grabWobble(grabWobbleSwitch.get())
            wobbleManipulator.upmediumdown(gamepad2.b, gamepad2.x) // correct
            /* Conveyor */
            conveyor.enableConveyor = conveyorOnOffSwitch.get()
            /* Shooter */
            shooter.shootingMode = if (shooterOnOffSwitch.get())
                if (shooterSpeedSwitch.get()) Shooter.ShooterMode.POWERSHOT
                else Shooter.ShooterMode.HIGHGOAL
            else Shooter.ShooterMode.OFF
            conveyor.forceReverse = gamepad2.right_trigger > 0.5
            if (gamepad1.a) shooter.feedRing()
            else if (threeRingPresser.get()) shooter.feedRings()
            /* Drivetrain */
            movement.humanSetVelocity(
                gamepad1.left_stick_x.toDouble() + if (gamepad1.dpad_left) -1.0 else 0.0 + if (gamepad1.dpad_right) 1.0 else 0.0,
                -gamepad1.left_stick_y.toDouble() + if (gamepad1.dpad_up) 1.0 else 0.0 + if (gamepad1.dpad_down) -1.0 else 0.0,
                if (gamepad1.right_bumper) 0.25 else gamepad1.right_trigger.toDouble() - if (gamepad1.left_bumper) 0.25 else gamepad1.left_trigger.toDouble()
            )
        }
    }

    /*if (gamepad2.left_bumper) {
     ShootHighGoalAsync()
    }
    if (!movement.pathFollowerIsActive()) {}
    if (gamepad1.x) ShootPOWERSHOTAngle()*/
}