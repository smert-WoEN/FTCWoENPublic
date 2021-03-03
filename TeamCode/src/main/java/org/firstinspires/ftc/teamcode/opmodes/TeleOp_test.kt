package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.robot.Shooter

@TeleOp
class TeleOp_test : LinearOpMode() {
    override fun runOpMode() {
        forceInitRobot(this)
        startRobot()
        odometry.robotCoordinates = Pose2D(0.0, 0.0, 0.0)
        /* Wobble */
        val grabWobbleSwitch = ButtonSwitch { gamepad1.a }
        /* Conveyor */
        val conveyorOnOffSwitch = ButtonSwitch { gamepad1.back }
        /* Shooter */
        val shooterOnOffSwitch = ButtonSwitch { gamepad1.start }
        val threeRingPresser = SinglePressButton { gamepad1.right_stick_button }
        while (opModeIsActive()) {
            /* Wobble */
            wobbleManipulator.grabWobble(grabWobbleSwitch.get())
            wobbleManipulator.upmediumdown(gamepad1.y, gamepad1.x) // correct
            /* Conveyor */
            WoENrobot.conveyor.enableConveyor = conveyorOnOffSwitch.get()
            /* Shooter */
            shooter.shootingMode =
                if (shooterOnOffSwitch.get()) Shooter.ShooterMode.HIGHGOAL else Shooter.ShooterMode.OFF
            if (gamepad1.b) shooter.feedRing()
            if (threeRingPresser.get()) shooter.feedRings()
            /* Drivetrain */
            movement.humanSetVelocity(
                gamepad1.left_stick_x.toDouble() + if (gamepad1.dpad_left) -1.0 else 0.0 + if (gamepad1.dpad_right) 1.0 else 0.0,
                -gamepad1.left_stick_y.toDouble() + if (gamepad1.dpad_up) 1.0 else 0.0 + if (gamepad1.dpad_down) -1.0 else 0.0,
                if (gamepad1.right_bumper) 0.25 else gamepad1.right_trigger.toDouble() - if (gamepad1.left_bumper) 0.25 else gamepad1.left_trigger.toDouble()
            )
        }
    }
}