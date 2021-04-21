package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.ServoWobbleManipulator.WobbleServoPositions.angleDown
import org.firstinspires.ftc.teamcode.robot.ServoWobbleManipulator.WobbleServoPositions.angleUp
import org.firstinspires.ftc.teamcode.robot.ServoWobbleManipulator.WobbleServoPositions.gripperClose
import org.firstinspires.ftc.teamcode.robot.ServoWobbleManipulator.WobbleServoPositions.gripperOpen
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.feederClose
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.feederOpen
import org.firstinspires.ftc.teamcode.robot.WoENHardware.conveyorMotor
import org.firstinspires.ftc.teamcode.robot.WoENHardware.driveFrontLeft
import org.firstinspires.ftc.teamcode.robot.WoENHardware.driveFrontRight
import org.firstinspires.ftc.teamcode.robot.WoENHardware.driveRearLeft
import org.firstinspires.ftc.teamcode.robot.WoENHardware.driveRearRight
import org.firstinspires.ftc.teamcode.robot.WoENHardware.feeder
import org.firstinspires.ftc.teamcode.robot.WoENHardware.gripper
import org.firstinspires.ftc.teamcode.robot.WoENHardware.leverArm
import org.firstinspires.ftc.teamcode.robot.WoENHardware.odometerYL
import org.firstinspires.ftc.teamcode.robot.WoENHardware.odometerYR
import org.firstinspires.ftc.teamcode.robot.WoENHardware.ringDetector
import org.firstinspires.ftc.teamcode.robot.WoENHardware.shooterMotor
import org.firstinspires.ftc.teamcode.robot.WoENrobot.ai
import org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot

@TeleOp
class Diagnostic : LinearOpMode() {
    override fun runOpMode() {
        forceInitRobot(this)
        startRobot()
        telemetry.addData("shooter", ai.diagnosticMotor(shooterMotor))
        telemetry.addData("Conveyor", ai.diagnosticMotor(conveyorMotor))
        telemetry.addData("OdometerYL", ai.diagnosticMotor(odometerYL))
        telemetry.addData("odometerYR", ai.diagnosticMotor(odometerYR))
        telemetry.addData("driveRearLeft", ai.diagnosticMotor(driveRearLeft))
        telemetry.addData("driveRearRight", ai.diagnosticMotor(driveRearRight))
        telemetry.addData("driveFrontRight", ai.diagnosticMotor(driveFrontRight))
        telemetry.addData("driveFrontLeft", ai.diagnosticMotor(driveFrontLeft))
        telemetry.addData("RingDetector", ai.diagnosticRange(ringDetector))
        telemetry.addData("See Servo", "")
        telemetry.update()
        ai.diagnosticServo(feeder, feederOpen, feederClose)
        ai.diagnosticServo(leverArm, angleDown, angleUp)
        ai.diagnosticServo(gripper, gripperOpen, gripperClose)
    }
}