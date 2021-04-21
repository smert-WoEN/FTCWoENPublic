package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter
import org.firstinspires.ftc.teamcode.misc.RegulatorPIDVAS
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kA
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kD
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kI
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kP
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kS
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kV
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.kV_referenceVoltage
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.maxI
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.motorControllerMode
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.DrivetrainConfig.secondsToAccelerate
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.VelocityOdometry
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

class MecanumDrivetrain(private val voltageSupplier: VoltageSupplier) : MultithreadedRobotModule(), Drivetrain, VelocityOdometry {
    /* Motor parameters constants. */
    @Config
    internal object DrivetrainConfig {
        @JvmField var achievableMaxVeloictyFraction = 0.885
        @JvmField var achievableMinVelocityFraction = 0.07
        @JvmField var strafingMultiplier = 1.3
        @JvmField var rotationDiscrepancy = 2.8
        @JvmField var motorControllerMode = MotorControllerMode.EXTERNAL_PID
        @JvmField var secondsToAccelerate = 0.10
        @JvmField var kP = 27.0
        @JvmField var kD = 0.0
        @JvmField var kI = 0.5
        @JvmField var kV = 14.46
        @JvmField var kA = .0
        @JvmField var kS = 1400.0
        @JvmField var maxI = 32767.0
        @JvmField var kV_referenceVoltage = 13.0
    }

    /* Physical constants */
    private val wheelRadius = 9.8 / 2.0
    private val gearRatio = (1.0 / 20.0) * (17.0 / 13.0)
    private val maxRPM = 6000.0
    private val encoderCPR = 24
    private val radiansToEncoderTicksMultiplier = encoderCPR / (PI * 2)
    private val wheelCenterOffset = Vector2D(18.05253, 15.20000)
    private val forwardMultiplier = radiansToEncoderTicksMultiplier / (wheelRadius * gearRatio)
    private val theoreticalMaxTickVelocity = (maxRPM / 60.0) * encoderCPR
    private val sidewaysMultiplier
        get() = forwardMultiplier * DrivetrainConfig.strafingMultiplier
    private val turnMultiplier
        get() = radiansToEncoderTicksMultiplier * (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDiscrepancy / (wheelRadius * gearRatio)
    private val maxMotorTickVelocity
        get() = DrivetrainConfig.achievableMaxVeloictyFraction * theoreticalMaxTickVelocity
    private val minMotorTickVelocity
        get() = DrivetrainConfig.achievableMinVelocityFraction * theoreticalMaxTickVelocity

    /* Drivetrain hardware members. */
    private lateinit var driveFrontLeft: DcMotorEx
    private lateinit var driveFrontRight: DcMotorEx
    private lateinit var driveRearLeft: DcMotorEx
    private lateinit var driveRearRight: DcMotorEx

    private var mFLPower = .0
    private var mFRPower = .0
    private var mRLPower = .0
    private var mRRPower = .0
    /* Motor controllers */
    private val mFLPowerSender = CommandSender({ driveFrontLeft.power = it })
    private val mFLVelocitySender = CommandSender({ driveFrontLeft.velocity = it })
    private val mFLReg = RegulatorPIDVAS({ mFLPower = it }, { measuredVelocityFL }, { voltageSupplier.voltage },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kP else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kI else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kD else 0.0 }, { kV }, { kA },
                                         { kS }, { maxI }, { kV_referenceVoltage })
    private val mFLProfiler = MotorAccelerationLimiter({ mFLReg.update(it) }, {theoreticalMaxTickVelocity / secondsToAccelerate})

    private val mFRPowerSender = CommandSender({ driveFrontRight.power = it })
    private val mFRVelocitySender = CommandSender({ driveFrontRight.velocity = it })
    private val mFRReg = RegulatorPIDVAS({ mFRPower = it }, { measuredVelocityFR }, { voltageSupplier.voltage },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kP else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kI else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kD else 0.0 }, { kV }, { kA },
                                         { kS }, { maxI }, { kV_referenceVoltage })
    private val mFRProfiler = MotorAccelerationLimiter({ mFRReg.update(it)}, {theoreticalMaxTickVelocity / secondsToAccelerate})

    private val mRLPowerSender = CommandSender({ driveRearLeft.power = it })
    private val mRLVelocitySender = CommandSender({ driveRearLeft.velocity = it })
    private val mRLReg = RegulatorPIDVAS({ mRLPower = it }, { measuredVelocityRL }, { voltageSupplier.voltage },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kP else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kI else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kD else 0.0 }, { kV }, { kA },
                                         { kS }, { maxI }, { kV_referenceVoltage })
    private val mRLProfiler = MotorAccelerationLimiter({ mRLReg.update(it)}, {theoreticalMaxTickVelocity / secondsToAccelerate})

    private val mRRPowerSender = CommandSender({ driveRearRight.power = it })
    private val mRRVelocitySender = CommandSender({ driveRearRight.velocity = it })
    private val mRRReg = RegulatorPIDVAS({ mRRPower = it }, { measuredVelocityRR }, { voltageSupplier.voltage },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kP else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kI else 0.0 },
                                         { if (motorControllerMode == MotorControllerMode.EXTERNAL_PID) kD else 0.0 }, { kV }, { kA },
                                         { kS }, { maxI }, { kV_referenceVoltage })
    private val mRRProfiler = MotorAccelerationLimiter({ mRRReg.update(it)}, {theoreticalMaxTickVelocity / secondsToAccelerate})

    override var targetVelocity = Vector3D(.0, .0, .0)

    private var targetTickVelocityFL = 0.0
    private var targetTickVelocityFR = 0.0
    private var targetTickVelocityRL = 0.0
    private var targetTickVelocityRR = 0.0
    private var measuredVelocityFL = 0.0
    private var measuredVelocityFR = 0.0
    private var measuredVelocityRL = 0.0
    private var measuredVelocityRR = 0.0
    override fun initialize() {
        assignHardware()
        setMotorDirections()
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        when (motorControllerMode) {
            MotorControllerMode.FEEDFORWARD -> {
                setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            }
            MotorControllerMode.EXTERNAL_PID -> {
                setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            }
            MotorControllerMode.INTERNAL_PID -> {
                setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER)
                try {
                    setInternalPIDFCoefficients(PIDFCoefficients(kP, kI, kD, kV))
                } catch (e: UnsupportedOperationException) {
                    opMode.telemetry.addData("Drivetrain PIDF error ", e.message)
                    e.printStackTrace()
                }
            }
        }
        setMotor0PowerBehaviors(DcMotor.ZeroPowerBehavior.FLOAT)
        targetVelocity = Vector3D(.0, .0, .0)
    }

    private fun assignHardware() {
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

    private fun setMotor0PowerBehaviors(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        driveFrontLeft.zeroPowerBehavior = zeroPowerBehavior
        driveFrontRight.zeroPowerBehavior = zeroPowerBehavior
        driveRearLeft.zeroPowerBehavior = zeroPowerBehavior
        driveRearRight.zeroPowerBehavior = zeroPowerBehavior
    }

    private fun setMotorMode(runMode: DcMotor.RunMode) {
        driveFrontLeft.mode = runMode
        driveFrontRight.mode = runMode
        driveRearLeft.mode = runMode
        driveRearRight.mode = runMode
    }

    private fun setInternalPIDFCoefficients(pidfCoefficients: PIDFCoefficients) {
        driveFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
    }

    override fun updateControlHub() {
        measuredVelocityFL = driveFrontLeft.velocity
        measuredVelocityFR = driveFrontRight.velocity
        measuredVelocityRL = driveRearLeft.velocity
        measuredVelocityRR = driveRearRight.velocity

        robotVelocity = Vector3D(
            (measuredVelocityFL - measuredVelocityFR - measuredVelocityRL - measuredVelocityRR) / (4 * sidewaysMultiplier),
            (measuredVelocityFL + measuredVelocityFR + measuredVelocityRL + measuredVelocityRR) / (4 * forwardMultiplier),
            (measuredVelocityFL - measuredVelocityFR + measuredVelocityRL - measuredVelocityRR) / (4 * turnMultiplier))

        val targetChassisVelocityTicks = Vector3D(targetVelocity.x * sidewaysMultiplier, targetVelocity.y * forwardMultiplier,
                                                  targetVelocity.z * turnMultiplier)
        targetTickVelocityFL = targetChassisVelocityTicks.y + targetChassisVelocityTicks.x + targetChassisVelocityTicks.z
        targetTickVelocityFR = targetChassisVelocityTicks.y - targetChassisVelocityTicks.x - targetChassisVelocityTicks.z
        targetTickVelocityRL = targetChassisVelocityTicks.y - targetChassisVelocityTicks.x + targetChassisVelocityTicks.z
        targetTickVelocityRR = targetChassisVelocityTicks.y + targetChassisVelocityTicks.x - targetChassisVelocityTicks.z

        var maxabs = abs(targetTickVelocityFL).coerceAtLeast(abs(targetTickVelocityFR)).coerceAtLeast(abs(targetTickVelocityRL))
            .coerceAtLeast(abs(targetTickVelocityRR))
        if (maxabs > maxMotorTickVelocity) {
            maxabs /= maxMotorTickVelocity
            targetTickVelocityFL /= maxabs
            targetTickVelocityFR /= maxabs
            targetTickVelocityRL /= maxabs
            targetTickVelocityRR /= maxabs
        }
        targetTickVelocityFL = limitMinSpeed(targetTickVelocityFL)
        targetTickVelocityFR = limitMinSpeed(targetTickVelocityFR)
        targetTickVelocityRL = limitMinSpeed(targetTickVelocityRL)
        targetTickVelocityRR = limitMinSpeed(targetTickVelocityRR)

        when (motorControllerMode) {
            MotorControllerMode.FEEDFORWARD -> {
                mFLReg.update(targetTickVelocityFL)
                mFRReg.update(targetTickVelocityFR)
                mRLReg.update(targetTickVelocityRL)
                mRRReg.update(targetTickVelocityRR)
            }
            MotorControllerMode.EXTERNAL_PID -> {
                mFLProfiler.setVelocity(targetTickVelocityFL)
                mFRProfiler.setVelocity(targetTickVelocityFR)
                mRLProfiler.setVelocity(targetTickVelocityRL)
                mRRProfiler.setVelocity(targetTickVelocityRR)
                mFLPowerSender.send(mFLPower)
                mFRPowerSender.send(mFRPower)
                mRLPowerSender.send(mRLPower)
                mRRPowerSender.send(mRRPower)
            }
            MotorControllerMode.INTERNAL_PID -> {
                mFLVelocitySender.send(targetTickVelocityFL)
                mFRVelocitySender.send(targetTickVelocityFR)
                mRLVelocitySender.send(targetTickVelocityRL)
                mRRVelocitySender.send(targetTickVelocityRR)
            }
        }
    }

    private fun limitMinSpeed(speed: Double): Double {
        return abs(speed).coerceAtLeast(minMotorTickVelocity) * sign(speed)
    }

    override val maxVelocity: Vector3D
        get() = Vector3D(maxMotorTickVelocity / forwardMultiplier, maxMotorTickVelocity / forwardMultiplier,
                         maxMotorTickVelocity / turnMultiplier)
    override var robotVelocity: Vector3D = Vector3D()

    enum class MotorControllerMode {
        FEEDFORWARD, EXTERNAL_PID, INTERNAL_PID
    }
}