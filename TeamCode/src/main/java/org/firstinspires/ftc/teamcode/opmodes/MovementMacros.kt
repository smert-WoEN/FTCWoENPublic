package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.HighGoalShootingAngle
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.HighGoalShootingDistance
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.ParkLineY
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.ParkingTolerance
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.PartnerWobblePoseYOffset
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.PowerShotShootingAngle
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.PowerShotShootingDistance
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.PowerShotShootingOffset
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.RingStackApproachOffset
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.RingStackFirstRingOffset
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.RingStackFirstRingOffsetFromFour
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.RingStackFourthRingOffset
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.RobotYBackLength
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.RobotYFrontLength
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MovementMacrosConfig.WobblePlacementOffset
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.StackSize
import org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor
import org.firstinspires.ftc.teamcode.robot.WoENrobot.delay
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.opMode
import org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.superclasses.MotionTask
import org.firstinspires.ftc.teamcode.superclasses.Shooter
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.abs

object MovementMacros {

    private var xSign: Int = 1
    private var sideSign: Int = 1

    fun setSettings(xSign: Int, sideSign: Int) {
        MovementMacros.xSign = xSign
        MovementMacros.sideSign = sideSign
    }

    @Config
    internal object MovementMacrosConfig {
        @JvmField var WobblePlacementOffset = Vector2D(11.8425, 39.25) //Vector2D(11.8425,33.25);
        @JvmField var PartnerWobblePoseYOffset = 0.0
        @JvmField var HighGoalShootingDistance = 204.0
        @JvmField var HighGoalShootingAngle = -8.3
        @JvmField var PowerShotShootingDistance = 200.0
        @JvmField var PowerShotShootingAngle = -6.69
        @JvmField var RingStackApproachOffset = 30.0
        @JvmField var RingStackFirstRingOffset = 5.0
        @JvmField var RingStackFirstRingOffsetFromFour = -2.0
        @JvmField var RingStackFourthRingOffset = -20.0
        @JvmField var ParkLineY = 26.462
        @JvmField var RobotYBackLength = 29.85498
        @JvmField var RobotYFrontLength = 37.2
        @JvmField var ParkingTolerance = 5.0
        @JvmField var PowerShotShootingOffset = -0.0
        //TODO Servo travel times
    }

    fun avoidRingStack() {
        movement.pos(Pose2D(odometry.robotCoordinates.x + 30 * sideSign, -60.0, Double.NaN), distanceTolerance = 20.0,
                     angularTolerance = .5)
    }

    /**
    /* Wobble */
     */
    private val wobblePose: Vector2D
        get() = when (openCVNode.stackSize) {
             StackSize.FOUR -> Vector2D(xSign * 150.3809, 150.2761)
             StackSize.ONE -> Vector2D(xSign * 89.9835, 90.3346)
            else -> Vector2D(xSign * 150.3809, 30.1596)
        }

    private val partnerWobblePose: Vector2D
        get() = Vector2D(90.91741046 * xSign - 30.1416 * sideSign, -136.3139 + PartnerWobblePoseYOffset)


    fun moveWobble() {
        wobbleManipulator.grabWobble(true)
        wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
        val wobblePose = wobblePose
        val error: Vector2D = movement.getError(Pose2D(wobblePose, Double.NaN))
        movement.followPath(MotionTask(wobblePose.minus(Vector2D(0.0, WobblePlacementOffset.radius()).rotatedCW(error.aCot())),
                                       error.aCot() - WobblePlacementOffset.aCot()), distanceTolerance = 4.0,
                            angularTolerance = toRadians(4.0))
        val leverServoTimer = ElapsedTime()
        wobbleManipulator.setAngle(WobbleManipulator.Position.DOWN)
        while (movement.pathFollowerIsActive() && opMode.opModeIsActive()) {
            if (movement.getError(Pose2D(wobblePose, Double.NaN)).radius() < 90) wobbleManipulator.setAngle(
                 WobbleManipulator.Position.DOWN)
            else leverServoTimer.reset()
            spinOnce()
        }

        //delay(500.0 - leverServoTimer.milliseconds())
        wobbleManipulator.grabWobble(false)
        delay(300.0)
    }

    fun pickSecondWobble() {
        wobbleManipulator.grabWobble(false)
        wobbleManipulator.setAngle(WobbleManipulator.Position.DOWN)
        val wobblePose = partnerWobblePose
        val error: Vector2D = movement.getError(Pose2D(wobblePose, Double.NaN))
        movement.pos(Pose2D(Double.NaN, Double.NaN, error.aCot()), angularTolerance = toRadians(10.0))
        movement.followPath(MotionTask(wobblePose.minus(Vector2D(0.0, WobblePlacementOffset.radius()).rotatedCW(error.aCot())),
                                       error.aCot() - WobblePlacementOffset.aCot()), distanceTolerance = 1.5,
                            angularTolerance = toRadians(1.0))
        while (movement.pathFollowerIsActive() && opMode.opModeIsActive()) {
            if (movement.getError(Pose2D(wobblePose, Double.NaN)).radius() < 75) wobbleManipulator.setAngle(
                 WobbleManipulator.Position.DOWN)
            spinOnce()
        }
        delay(200.0)
        wobbleManipulator.grabWobble(true)
        delay(300.0)
        movement.pos(Pose2D(Double.NaN, Double.NaN, 0.0), angularTolerance = toRadians(10.0))
    }

    fun shooting(NeedAngle: Boolean) {
        when {
            sideSign * xSign == 1 -> shootHighGoal()
            NeedAngle -> shootPOWERSHOTAngle()
            else -> shootPowerShotPos()
        }
    }

    fun shooting() {
        when {
            sideSign * xSign == 1 -> {
                shootHighGoal()
            //    moveWobble()
            }
            else -> {
                shootPowerShotDynamic()
              //  pickupRingsAfterPowerShots()
               // moveWobble()
                //shootHighGoal()
            }
        }
    }

    /**
    /* High Goal */
     */
    private val highGoalPose: Vector2D
        get() = Vector2D(93.9174 * xSign, 182.691)

    private val highGoalShootingPose: Pose2D
        get() {
            val error = movement.getError(Pose2D(highGoalPose, Double.NaN))
            val angle = Range.clip(error.aCot(), toRadians(-13.0), toRadians(13.0))
            return Pose2D(highGoalPose - Vector2D(0.0, HighGoalShootingDistance).rotatedCW(angle),
                          angle + toRadians(HighGoalShootingAngle))
        }

    fun shootHighGoal(shootOnlyOneRing: Boolean = false) {
        wobbleManipulator.setAngle(WobbleManipulator.Position.UP)
        shooter.shootingMode = Shooter.ShooterMode.HIGHGOAL
        movement.pos(highGoalShootingPose)
        val shooterAccelerationTimeout = ElapsedTime()
        while (opMode.opModeIsActive() && !shooter.isCorrectRpm(100.0) && shooterAccelerationTimeout.seconds() < 1.2) spinOnce()
        shooter.feedRings()
        delay(if (shootOnlyOneRing) shooter.timeToShootOneRing else shooter.timeToShootThreeRings)
        shooter.shootingMode = Shooter.ShooterMode.OFF
    }


    fun shootHighGoalAsync() {
       //. shooter.shootingMode = Shooter.ShooterMode.HIGHGOAL
        movement.followPath(MotionTask(highGoalShootingPose) {
            val shooterAccelerationTimeout = ElapsedTime()
            while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds() < 1.2 && movement.pathFollowerIsActive())
                spinOnce()
            //if (movement.pathFollowerIsActive())
        shooter.feedRings()
        })
        //  while(movement.pathFollowerIsActive()&&getOpMode().opModeIsActive()) {spinOnce();}
    }


    enum class PowerShot {
        LEFT, MEDIUM, RIGHT
    }

    private fun powerShotPose(powerShot: PowerShot): Vector2D {
        return when (powerShot) {
             PowerShot.LEFT -> Vector2D(9.600 * xSign, 182.691)
             PowerShot.MEDIUM -> Vector2D(28.595 * xSign, 182.691)
             PowerShot.RIGHT -> Vector2D(47.603 * xSign, 182.691)
        }
    }

    private fun powerShotShootingPose(powerShot: PowerShot): Pose2D {
        val heading = Range.clip(movement.getError(Pose2D(powerShotPose(PowerShot.MEDIUM), Double.NaN)).aCot(), toRadians(-17.0),
                                 toRadians(17.0))
        val shootingCoordinates = powerShotPose(PowerShot.MEDIUM).minus(Vector2D(-PowerShotShootingOffset, PowerShotShootingDistance).rotatedCW(heading))
        return when (powerShot) {
             PowerShot.LEFT -> Pose2D(shootingCoordinates,
                                      heading + toRadians(PowerShotShootingAngle) + powerShotPose(PowerShot.LEFT).minus(
                                           shootingCoordinates).aCot() - powerShotPose(PowerShot.MEDIUM).minus(shootingCoordinates)
                                           .aCot())
             PowerShot.MEDIUM -> Pose2D(shootingCoordinates, heading + toRadians(PowerShotShootingAngle))
             PowerShot.RIGHT -> Pose2D(shootingCoordinates,
                                       heading + toRadians(PowerShotShootingAngle) + powerShotPose(PowerShot.RIGHT).minus(
                                            shootingCoordinates).aCot() - powerShotPose(PowerShot.MEDIUM).minus(shootingCoordinates)
                                            .aCot())
        }
    }

    fun shootPowerShotDynamic() {
        wobbleManipulator.setAngle(WobbleManipulator.Position.UP)
        shooter.shootingMode = Shooter.ShooterMode.POWERSHOT
        for (powerShot in PowerShot.values()) {
            movement.pos(powerShotShootingPose(powerShot))
            val shooterAccelerationTimeout = ElapsedTime()
            shooterAccelerationTimeout.reset()
            while (opMode.opModeIsActive() && shooterAccelerationTimeout.seconds() < 2 && (!shooter.isCorrectRpm()  || abs(odometry.robotVelocity.z)>0.1)) spinOnce()
            shooter.feedRing()
            delay(shooter.timeToShootOneRing)
        }
        shooter.shootingMode = Shooter.ShooterMode.OFF
    }


    fun shootPOWERSHOTAngle() {  //rename
        shooter.shootingMode = Shooter.ShooterMode.POWERSHOT
        val pos = 31.0
        var angle = 6.5
        for (i in 0..2) {
            if (xSign == 1) {
                movement.pos(Pose2D(xSign * pos, -7.5, toRadians(angle)))
            } else {
                movement.pos(Pose2D(xSign * (pos + 5), -7.5, toRadians(-angle)))
            }
            angle -= 5.4
            delay(200.0)
            if (opMode.gamepad1.x) {
                break
            }
            //pos -= 18;
            // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
            // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing()
            delay(200.0)
        }
        shooter.shootingMode = Shooter.ShooterMode.OFF
    }

    fun shootPowerShotPos() {  //rename
        shooter.shootingMode = Shooter.ShooterMode.POWERSHOT
        var pos = 50.0
        val angle = 5.5
        for (i in 0..2) {
            if (xSign == 1) movement.pos(Pose2D(xSign * pos, -5.0, toRadians(angle)))
            else movement.pos(Pose2D(xSign * pos, -5.0, toRadians(3.0)))
            //angle -= 6.4;
            if (opMode.gamepad1.x) break
            pos -= 18.0
            // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
            // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing()
            delay(200.0)
        }
        shooter.shootingMode = Shooter.ShooterMode.OFF
    }

    /*
    // Ring stack
    */

    private val ringStackPose: Vector2D
        get() = Vector2D(90.3747 * xSign - 8.5, -56.9019)

    fun pickupRings(pickWobbleBetweenRings: Boolean = false) {
        val heading = movement.getError(Pose2D(ringStackPose, Double.NaN)).aCot()
        when (openCVNode.stackSize) {
             StackSize.FOUR -> {
                  movement.pos(Pose2D(ringStackPose - Vector2D(0.0, RingStackApproachOffset).rotatedCW(heading), heading + PI),
                               distanceTolerance = 25.0, angularTolerance = toRadians(5.0))
                  conveyor.enableConveyor = true
                  movement.pos(Pose2D(ringStackPose - Vector2D(0.0, RingStackFirstRingOffsetFromFour).rotatedCW(heading), heading + PI),
                               linearVelocityFraction = .16, distanceTolerance = 5.0, angularTolerance = toRadians(5.0))
                  //delay(700.0)
                  shootHighGoal()
                  movement.pos(Pose2D(ringStackPose - Vector2D(0.0, RingStackFourthRingOffset).rotatedCW(heading), heading + PI),
                               linearVelocityFraction = .35, distanceTolerance = 4.0, angularTolerance = toRadians(4.0))
                  if (pickWobbleBetweenRings) pickSecondWobble()
                  ///else delay(750.0)
                  shootHighGoal()
             }
             StackSize.ONE -> {
                  conveyor.enableConveyor = true
                  movement.pos(Pose2D(ringStackPose - Vector2D(0.0, RingStackFirstRingOffset).rotatedCW(heading), heading + PI),
                               linearVelocityFraction = .7, distanceTolerance = 4.0, angularTolerance = toRadians(4.0))
                  if (pickWobbleBetweenRings) pickSecondWobble()
                  //else delay(500.0)
                  shootHighGoal()
             }
             StackSize.ZERO -> {
                  if (pickWobbleBetweenRings) pickSecondWobble()
             }
        }
        conveyor.enableConveyor = false
    }

    /*
    /* Parking */
    */

    fun park() {
        if (odometry.robotCoordinates.y > ParkLineY) {
            movement.pos(Pose2D(89.6372 * xSign + 67.3092 * sideSign, ParkLineY + RobotYBackLength - ParkingTolerance, 0.0),
                         distanceTolerance = ParkingTolerance, angularTolerance = toRadians(5.0))
        } else {
            wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
            movement.pos(Pose2D(89.6372 * xSign + 67.3092 * sideSign, ParkLineY - RobotYFrontLength + ParkingTolerance, 0.0),
                         distanceTolerance = ParkingTolerance, angularTolerance = toRadians(5.0))
            delay(20.0)
        }
    }
    private val ringsPowerShot: Vector2D
        get() = Vector2D(-10.0 * xSign, 130.0)
    fun pickupRingsAfterPowerShots() {
        movement.pos(Pose2D(ringsPowerShot, -PI / 2 * xSign))
        conveyor.enableConveyor = true
        movement.pos(Pose2D(ringsPowerShot + Vector2D(60.0 * xSign, 0.0), -PI / 2 * xSign), linearVelocityFraction = 0.3)
        conveyor.enableConveyor = false
    }
    /*
        @Deprecated("")
        fun shootTargets() {
            //shooter.setShooterSettings(3850, 500);
            shooter.shootingMode = rpm.ShooterMode.HIGHGOAL
            if (sideSign.toInt() == 1 && xSign.toInt() == 1)
                movement.pos(Pose2D((xSign * 121).toDouble(), -48.5, toRadians(-8.5)))
            else if (sideSign.toInt() == -1 && xSign.toInt() == 1)
                movement.pos(Pose2D((xSign * 53).toDouble(), -30.0, toRadians(3.0)))
            else if (sideSign.toInt() == -1 && xSign.toInt() == -1) movement.pos(Pose2D(xSign * 147.5, -9.5, toRadians(10.5)))
            else  // if (sideSign == 1 &&  xSign == -1)
                movement.pos(Pose2D(xSign * 61.5, -28.0, toRadians(-11.5)))
            val shooterAccelerationTimeout = ElapsedTime()
            while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds() < 3)
                spinOnce()
            shooter.feedRings()
            delay(900.0)
            shooter.shootingMode = rpm.ShooterMode.OFF
        }

        @Deprecated("")
        fun putRingsToLowGoal() {
            movement.pos(Pose2D((93.75 + 11.25 * xSign) * xSign, 150.0, toRadians(0.0)))
            wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
            delay(1000.0)
            wobbleManipulator.setAngle(WobbleManipulator.Position.UP)
        }*/

}