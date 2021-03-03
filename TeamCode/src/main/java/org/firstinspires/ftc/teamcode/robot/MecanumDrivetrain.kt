package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.WoENHardware.controlHubVoltageSensor
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import kotlin.math.abs
import kotlin.math.sign

class MecanumDrivetrain : MultithreadRobotModule(), Drivetrain {
    /* Motor parameters constatnts. */
    @Config
    internal object DrivetrainConfig {
        @JvmField
        var achieveableMaxRPMFraction = 0.885
        @JvmField
        var achieveableMinRPMFraction = 0.045
        @JvmField
        var strafingMultiplier = 1.35
        @JvmField
        var rotationDecrepancy = 1.0
        @JvmField
        var secondsToAccelerate = 0.33
        @JvmField
        var kP = 26.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kI = 0.1
        @JvmField
        var kF = 15.10
        @JvmField
        var kF_referenceVoltage = 13.0
    }

    /* Physical constants */
    private val wheelRadius = 9.8 / 2
    private val gearRatio = 17.0 / 13.0
    private val wheelCenterOffset = Vector2D(18.05253, 15.20000)
    private val forwardMultiplier = (1 / wheelRadius) / gearRatio
    private var sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
    private var turnMultiplier =
        (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
    private val tickPerRev = 480.0
    private val gearing = 20.0
    private val maxRPM = 300.0
    private val theoreticalMaxSpeed = maxRPM / 60 * Math.PI * 2
    private var maxMotorSpeed = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
    private var minMotorSpeed =
        DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed //http://b1-srv-kms-1.sch239.net:8239

    /* Drivetrain hardware members. */
    private lateinit var driveFrontLeft: DcMotorEx
    private lateinit var driveFrontRight: DcMotorEx
    private lateinit var driveRearLeft: DcMotorEx
    private lateinit var driveRearRight: DcMotorEx

    private var maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate

    /* Motor controllers */
    private val mFLSender = CommandSender { v: Double -> driveFrontLeft.setVelocity(v, AngleUnit.RADIANS) }
    private val mFLProfiler = motorAccelerationLimiter({ value: Double -> mFLSender.send(value) }, maxAcceleration)

    private val mFRSender = CommandSender { v: Double -> driveFrontRight.setVelocity(v, AngleUnit.RADIANS) }
    private val mFRProfiler = motorAccelerationLimiter({ value: Double -> mFRSender.send(value) }, maxAcceleration)

    private val mRLSender = CommandSender { v: Double -> driveRearLeft.setVelocity(v, AngleUnit.RADIANS) }
    private val mRLProfiler = motorAccelerationLimiter({ value: Double -> mRLSender.send(value) }, maxAcceleration)

    private val mRRSender = CommandSender { v: Double -> driveRearRight.setVelocity(v, AngleUnit.RADIANS) }
    private val mRRProfiler = motorAccelerationLimiter({ value: Double -> mRRSender.send(value) }, maxAcceleration)

    private lateinit var voltageSensor: VoltageSensor
    private var smartMode = false
    private var powerFrontLeft = 0.0
    private var powerFrontRight = 0.0
    private var powerRearLeft = 0.0
    private var powerRearRight = 0.0
    override fun initialize() {
        assignNames()
        voltageSensor = controlHubVoltageSensor
        setMotorDirections()
        maxMotorSpeed = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
        minMotorSpeed = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed
        sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
        maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate
        turnMultiplier =
            (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
        setMotor0PowerBehaviors(ZeroPowerBehavior.BRAKE)
        setMotorConfiguration(
            DrivetrainConfig.achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
        try {
            setPIDFCoefficients(
                PIDFCoefficients(
                    DrivetrainConfig.kP,
                    DrivetrainConfig.kD,
                    DrivetrainConfig.kI,
                    DrivetrainConfig.kF * DrivetrainConfig.kF_referenceVoltage / voltageSensor.voltage
                )
            )
        } catch (e: UnsupportedOperationException) {
            opMode.telemetry.addData("Drivetrain PIDF error ", e.message)
        }
        setSmartMode(true)
        setRobotVelocity(0.0, 0.0, 0.0)
    }

    fun setSmartMode(SmartMode: Boolean) {
        smartMode = SmartMode
        setMotorMode(if (SmartMode) RunMode.RUN_USING_ENCODER else RunMode.RUN_WITHOUT_ENCODER)
    }

    private fun assignNames() {
        driveFrontLeft = WoENHardware.driveFrontLeft
        driveFrontRight = WoENHardware.driveFrontRight
        driveRearLeft = WoENHardware.driveRearLeft
        driveRearRight = WoENHardware.driveRearRight
    }

    private fun setMotorDirections() {
        driveFrontLeft.direction = DcMotorSimple.Direction.FORWARD
        driveFrontRight.direction = DcMotorSimple.Direction.REVERSE
        driveRearLeft.direction = DcMotorSimple.Direction.FORWARD
        driveRearRight.direction = DcMotorSimple.Direction.REVERSE
    }

    private fun setMotor0PowerBehaviors(zeroPowerBehavior: ZeroPowerBehavior) {
        driveFrontLeft.zeroPowerBehavior = zeroPowerBehavior
        driveFrontRight.zeroPowerBehavior = zeroPowerBehavior
        driveRearLeft.zeroPowerBehavior = zeroPowerBehavior
        driveRearRight.zeroPowerBehavior = zeroPowerBehavior
    }

    private fun setMotorMode(runMode: RunMode) {
        driveFrontLeft.mode = runMode
        driveFrontRight.mode = runMode
        driveRearLeft.mode = runMode
        driveRearRight.mode = runMode
    }

    private fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients) {
        driveFrontLeft.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveFrontRight.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearLeft.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearRight.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
    }

    private fun setMotorConfiguration(
        achieveableMaxRPMFraction: Double,
        tickPerRev: Double,
        gearing: Double,
        maxRPM: Double
    ) {
        setMotorConfiguration(
            driveFrontLeft,
            achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
        setMotorConfiguration(
            driveFrontRight,
            achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(
            driveRearRight,
            achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
    }

    private fun setMotorConfiguration(
        dcMotor: DcMotorEx,
        achieveableMaxRPMFraction: Double,
        tickPerRev: Double,
        gearing: Double,
        maxRPM: Double
    ) {
        val motorConfigurationType = dcMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = achieveableMaxRPMFraction
        motorConfigurationType.ticksPerRev = tickPerRev
        motorConfigurationType.gearing = gearing
        motorConfigurationType.maxRPM = maxRPM
        dcMotor.motorType = motorConfigurationType
    }

    private fun resetEncoders() {
        setMotorMode(RunMode.STOP_AND_RESET_ENCODER)
    }

    fun setMaxDriveSpeed(value: Double) {
        maxMotorSpeed = Range.clip(abs(value), 0.0, theoreticalMaxSpeed)
    }

    fun setMinDriveSpeed(value: Double) {
        minMotorSpeed = Range.clip(abs(value), 0.0, theoreticalMaxSpeed)
    }

    override fun updateControlHub() {
        if (smartMode) {
            mFLProfiler.setVelocity(powerFrontLeft)
            mFRProfiler.setVelocity(powerFrontRight)
            mRLProfiler.setVelocity(powerRearLeft)
            mRRProfiler.setVelocity(powerRearRight)
        } else driveMotorPowersDirect(
            powerFrontLeft / maxMotorSpeed,
            powerFrontRight / maxMotorSpeed,
            powerRearLeft / maxMotorSpeed,
            powerRearRight / maxMotorSpeed
        )
    }

    override fun updateExpansionHub() {
    }

    override fun updateOther() {
    }

    private fun driveMotorPowersDirect(
        frontLeft: Double,
        frontRight: Double,
        rearLeft: Double,
        rearRight: Double
    ) {
        driveFrontLeft.power = frontLeft
        driveFrontRight.power = frontRight
        driveRearLeft.power = rearLeft
        driveRearRight.power = rearRight
    }

    private fun driveMotorPowers(
        frontLeft: Double,
        frontRight: Double,
        rearLeft: Double,
        rearRight: Double
    ) {
        var frontLeftMotorVelocity = frontLeft
        var frontRightMotorVelocity = frontRight
        var rearLeftMotorVelocity = rearLeft
        var rearRightMotorVelocity = rearRight
        var maxabs = abs(frontLeftMotorVelocity).coerceAtLeast(abs(frontRightMotorVelocity))
            .coerceAtLeast(abs(rearLeftMotorVelocity)).coerceAtLeast(abs(rearRightMotorVelocity))
        if (maxabs > maxMotorSpeed) {
            maxabs /= maxMotorSpeed
            frontLeftMotorVelocity /= maxabs
            frontRightMotorVelocity /= maxabs
            rearLeftMotorVelocity /= maxabs
            rearRightMotorVelocity /= maxabs
        }
        powerFrontLeft = limitSpeed(frontLeftMotorVelocity)
        powerFrontRight = limitSpeed(frontRightMotorVelocity)
        powerRearLeft = limitSpeed(rearLeftMotorVelocity)
        powerRearRight = limitSpeed(rearRightMotorVelocity)
    }

    private fun limitSpeed(speed: Double): Double {
        return Range.clip(abs(speed), minMotorSpeed, maxMotorSpeed) * sign(speed)
    }

    override val maxVelocity: Vector3D
        get() = Vector3D(
            maxMotorSpeed / forwardMultiplier,
            maxMotorSpeed / forwardMultiplier,
            maxMotorSpeed / turnMultiplier
        )


    override fun setRobotVelocity(frontways: Double, sideways: Double, turn: Double) {
        val frontwaysMotorVelocity = frontways * forwardMultiplier
        val sidewaysMotorVelocity = sideways * sidewaysMultiplier
        val turnMotorVelocity = turn * turnMultiplier
        driveMotorPowers(
            frontwaysMotorVelocity + sidewaysMotorVelocity + turnMotorVelocity,
            frontwaysMotorVelocity - sidewaysMotorVelocity - turnMotorVelocity,
            frontwaysMotorVelocity - sidewaysMotorVelocity + turnMotorVelocity,
            frontwaysMotorVelocity + sidewaysMotorVelocity - turnMotorVelocity
        )
    }
}