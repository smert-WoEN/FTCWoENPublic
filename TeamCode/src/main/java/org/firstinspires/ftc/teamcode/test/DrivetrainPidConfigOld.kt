package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.robot.WoENHardware
import kotlin.math.abs

@TeleOp
@Disabled
@Deprecated("See DriveTrainTuner")
class DrivetrainPidConfigOld : LinearOpMode() {
    @Config
    @Disabled
    object Constants {
        @JvmField var achieveableMaxRPMFraction = 0.9

        @JvmField var achieveableMinRPMFraction = 0.05

        @JvmField var strafingMultiplier = 1 / 0.8

        @JvmField var rotationDecrepancy = 1.0

        @JvmField var kP = 1.5

        @JvmField var kD = 0.0

        @JvmField var kI = 0.15

        @JvmField var kF = 15.0
    }

    private val wheelRadius = 9.8 / 2
    private val wheelCenterOffset = Vector2D(18.05253, 15.20000)
    private val forwardMultiplier = 1 / wheelRadius
    private var sidewaysMultiplier = forwardMultiplier * Constants.strafingMultiplier
    private var turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * Constants.rotationDecrepancy / wheelRadius
    private val tickPerRev = 480.0
    private val gearing = 20.0
    private val maxRPM = 300.0
    private val theoreticalMaxSpeed = maxRPM / 60 * Math.PI * 2
    private var maxMotorSpeed = Constants.achieveableMaxRPMFraction * theoreticalMaxSpeed
    private var minMotorSpeed = Constants.achieveableMinRPMFraction * theoreticalMaxSpeed
    val odometry = ThreeWheelOdometry()

    lateinit var driveFrontLeft: DcMotorEx
    lateinit var driveFrontRight: DcMotorEx
    lateinit var driveRearLeft: DcMotorEx
    lateinit var driveRearRight: DcMotorEx
    fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients?) {
        driveFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
    }

    fun setRobotVelocity(frontways: Double, sideways: Double, turn: Double) {
        var frontWays = frontways
        var sideWays = sideways
        var Turn = turn
        frontWays *= forwardMultiplier
        sideWays *= sidewaysMultiplier
        Turn *= turnMultiplier
        val FrontLeft = frontWays + sideWays + turn
        val FrontRight = frontWays - sideWays - turn
        val RearLeft = frontWays - sideWays + turn
        val RearRight = frontWays + sideWays - turn
        driveMotorPowers(FrontLeft, FrontRight, RearLeft, RearRight)
    }

    fun driveMotorPowers(frontLeft: Double, frontRight: Double, rearLeft: Double, rearRight: Double) {
        var FrontLeft = frontLeft
        var FrontRight = frontRight
        var RearLeft = rearLeft
        var RearRight = rearRight
        var maxabs = abs(frontLeft).coerceAtLeast(abs(frontRight)).coerceAtLeast(abs(rearLeft).coerceAtLeast(abs(rearRight)))
        if (maxabs > maxMotorSpeed) {
            maxabs /= maxMotorSpeed
            FrontLeft /= maxabs
            FrontRight /= maxabs
            RearLeft /= maxabs
            RearRight /= maxabs
        }
        powerFrontLeft = limitSpeed(frontLeft)
        powerFrontRight = limitSpeed(frontRight)
        powerRearLeft = limitSpeed(rearLeft)
        powerRearRight = limitSpeed(rearRight)
    }

    private fun limitSpeed(speed: Double): Double {
        return Range.clip(Math.abs(speed), minMotorSpeed, maxMotorSpeed) * Math.signum(speed)
    }

    private fun setMotorConfiguration(achieveableMaxRPMFraction: Double, tickPerRev: Double, gearing: Double, maxRPM: Double) {
        setMotorConfiguration(driveFrontLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(driveFrontRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(driveRearRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
    }

    private fun setMotorConfiguration(dcMotor: DcMotorEx, achieveableMaxRPMFraction: Double, tickPerRev: Double, gearing: Double, maxRPM: Double) {
        val motorConfigurationType = dcMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = achieveableMaxRPMFraction
        motorConfigurationType.ticksPerRev = tickPerRev
        motorConfigurationType.gearing = gearing
        motorConfigurationType.maxRPM = maxRPM
        dcMotor.motorType = motorConfigurationType
    }

    private val maxVelocity: Vector3D
        get() = Vector3D(maxMotorSpeed / forwardMultiplier, maxMotorSpeed / forwardMultiplier, maxMotorSpeed / turnMultiplier)
    private var powerFrontLeft = 0.0
    private var powerFrontRight = 0.0
    private var powerRearLeft = 0.0
    private var powerRearRight = 0.0
    private val maxAcceleration = theoreticalMaxSpeed / 0.25
    private val mFLProfiler = MotorAccelerationLimiter(
         { value: Double -> CommandSender({ v: Double -> driveFrontLeft.setVelocity(v, AngleUnit.RADIANS) }).send(value) },
         maxAcceleration)
    private val mFRProfiler = MotorAccelerationLimiter(
         { value: Double -> CommandSender({ v: Double -> driveFrontRight.setVelocity(v, AngleUnit.RADIANS) }).send(value) },
         maxAcceleration)
    private val mRLProfiler = MotorAccelerationLimiter(
         { value: Double -> CommandSender({ v: Double -> driveRearLeft.setVelocity(v, AngleUnit.RADIANS) }).send(value) },
         maxAcceleration)
    private val mRRProfiler = MotorAccelerationLimiter(
         { value: Double -> CommandSender({ v: Double -> driveRearRight.setVelocity(v, AngleUnit.RADIANS) }).send(value) },
         maxAcceleration)
    lateinit var dashboard: FtcDashboard
    lateinit var allHubs: List<LynxModule>
    override fun runOpMode() {
        waitForStart()
        WoENHardware.assignHardware(hardwareMap)
        odometry.initialize(this)
        dashboard = FtcDashboard.getInstance()
        driveFrontLeft = hardwareMap.get(DcMotorEx::class.java, "driveFrontLeft")
        driveFrontRight = hardwareMap.get(DcMotorEx::class.java, "driveFrontRight")
        driveRearLeft = hardwareMap.get(DcMotorEx::class.java, "driveRearLeft")
        driveRearRight = hardwareMap.get(DcMotorEx::class.java, "driveRearRight")
        driveFrontLeft.direction = DcMotorSimple.Direction.FORWARD
        driveFrontRight.direction = DcMotorSimple.Direction.REVERSE
        driveRearLeft.direction = DcMotorSimple.Direction.FORWARD
        driveRearRight.direction = DcMotorSimple.Direction.REVERSE
        driveFrontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        driveFrontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        driveRearLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        driveRearRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        driveFrontLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        driveFrontRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
        driveRearLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        driveRearRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
        telemetry = MultipleTelemetry(dashboard.telemetry, telemetry)
        allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (module in allHubs) module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        setPIDFCoefficients(PIDFCoefficients(Constants.kP, Constants.kD, Constants.kI, Constants.kF))
        val sineResetter = SinglePressButton { gamepad1.b }
        val sineWaveTimer = ElapsedTime()
        telemetry.msTransmissionInterval = 40
        for (module in allHubs) module.clearBulkCache()
        odometry.updateAll()
        while (opModeIsActive()) {
            for (module in allHubs) module.clearBulkCache()
            // odometry.update();
            var targetVelocity = Vector3D(gamepad1.left_stick_x.toDouble(), (-gamepad1.left_stick_y).toDouble(),
                                          gamepad1.right_stick_x.toDouble()).times(maxVelocity)
            if (sineResetter.get()) sineWaveTimer.reset()
            if (gamepad1.b) targetVelocity = Vector3D(0.0, Math.sin(sineWaveTimer.seconds() * Math.PI / 3), 0.0).times(maxVelocity)
            telemetry.addData("targetX", targetVelocity.x)
            telemetry.addData("targety", targetVelocity.y)
            telemetry.addData("targetz", targetVelocity.z)
            setRobotVelocity(targetVelocity.y, targetVelocity.x, targetVelocity.z)
            val expectedVelocity = calculateEncoderDelta(powerFrontLeft, powerFrontRight, powerRearLeft, powerRearRight)
            telemetry.addData("commandX", expectedVelocity.x)
            telemetry.addData("commandY", expectedVelocity.y)
            telemetry.addData("commandZ", expectedVelocity.z)
            mFLProfiler.setVelocity(powerFrontLeft)
            mFRProfiler.setVelocity(powerFrontRight)
            mRLProfiler.setVelocity(powerRearLeft)
            mRRProfiler.setVelocity(powerRearRight)
            val wheelVelocity = calculateEncoderDelta(driveFrontLeft.getVelocity(AngleUnit.RADIANS),
                                                      driveFrontRight.getVelocity(AngleUnit.RADIANS),
                                                      driveRearLeft.getVelocity(AngleUnit.RADIANS),
                                                      driveRearRight.getVelocity(AngleUnit.RADIANS))
            telemetry.addData("wheelX", wheelVelocity.x)
            telemetry.addData("wheelY", wheelVelocity.y)
            telemetry.addData("wheelZ", wheelVelocity.z)
            if (gamepad1.back) {
                setPIDFCoefficients(PIDFCoefficients(Constants.kP, Constants.kD, Constants.kI, Constants.kF))
                setMotorConfiguration(Constants.achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
                maxMotorSpeed = Constants.achieveableMaxRPMFraction * theoreticalMaxSpeed
                minMotorSpeed = Constants.achieveableMinRPMFraction * theoreticalMaxSpeed
                sidewaysMultiplier = forwardMultiplier * Constants.strafingMultiplier
                turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * Constants.rotationDecrepancy / wheelRadius
            }
            val velocity = odometry.robotVelocity
            telemetry.addData("VelX", velocity.x)
            telemetry.addData("VelY", velocity.y)
            telemetry.addData("VelZ", velocity.z)
            telemetry.update()
        }
    }

    fun calculateEncoderDelta(frontLeft: Double, frontRight: Double, rearLeft: Double, rearRight: Double): Vector3D {
        return Vector3D((frontLeft - frontRight - rearLeft - frontRight) / (4 * sidewaysMultiplier),
                        (frontLeft + frontRight + rearLeft + rearRight) / (4 * forwardMultiplier),
                        (frontLeft - frontRight + rearLeft - rearRight) / (4 * turnMultiplier))
    }
}