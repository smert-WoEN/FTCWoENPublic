package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.RegulatorPIDVAS
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kA
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kD
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kI
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kP
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kS
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kV
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.kV_referenceVoltage
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.maxI
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.servoReturnMultiplier
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.servoTime
import org.firstinspires.ftc.teamcode.robot.Shooter.ShooterConfig.feederClose
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Shooter
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier
import org.openftc.revextensions2.ExpansionHubServo
import kotlin.math.abs

class Shooter(private val voltageSupplier: VoltageSupplier) : MultithreadedRobotModule(), Shooter {
    private val feederTime = ElapsedTime()

    @Config
    internal object ShooterConfig {
        @JvmField var servoTime = 59
        @JvmField var servoReturnMultiplier = 3.5
        @JvmField var lowRpm = 3000.0
        @JvmField var highRpm = 3600.0
        @JvmField var feederClose = 0.23
        @JvmField var feederOpen = 0.39
        @JvmField var kP = 95.0
        @JvmField var kI = 0.27
        @JvmField var kD = 0.0
        @JvmField var kV = 14.06
        @JvmField var kA = 2.0
        @JvmField var kS = 2400.0
        @JvmField var maxI = 8192.0
        @JvmField var kV_referenceVoltage = 12.485
    }

    val timeToShootOneRing: Double
    get() = servoTime * servoReturnMultiplier
    val timeToShootThreeRings: Double
    get() = timeToShootOneRing * 3

    private lateinit var shooterMotor: DcMotorEx
    private lateinit var feeder: ExpansionHubServo
    private val shooterPowerSender = CommandSender({shooterMotor.power = it})
    private val shooterRegulator = RegulatorPIDVAS({shooterPowerSender.send(it)}, {currentVelocity}, {voltageSupplier.voltage}, {kP}, {kI}, {kD}, {kV}, {kA}, {kS}, {maxI}, {kV_referenceVoltage}, false)
    private val feederPositionSender = CommandSender({feeder.position = it})
    private var shooterMode = Shooter.ShooterMode.OFF
    private var ringsToShoot: Int = 0
    private var currentVelocity = 0.0
    private val maxInt16 = 32767.0
    private val ticksToRpmMultiplier = 2.5
    private val maxRpm: Double
        get() = (maxInt16 - kS) * ticksToRpmMultiplier / kV
    var rpmTarget = 0.0
        private set(value) {
            field = Range.clip(value, 0.0, maxRpm)
        }
    private var motorVelocityTarget = 0.0
    var currentRpm = 0.0
    var encoderFailureMode = false
        private set

    override fun initialize() {
        shooterMotor = WoENHardware.shooterMotor
        shooterMotor.direction = DcMotorSimple.Direction.FORWARD
        shooterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooterMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shootingMode = Shooter.ShooterMode.OFF
        initializedservo()
        feederTime.reset()
    }

    private fun initializedservo() {
        feeder = opMode.hardwareMap.get(Servo::class.java, "feeder") as ExpansionHubServo
        feeder.position = feederClose
    }

    override fun start() {
        feeder.position = feederClose
        shooterMotor.power = 0.0
        shootingMode = Shooter.ShooterMode.OFF
        ringsToShoot = 0
    }

    override fun updateControlHub() {
        if (ringsToShoot > 0 && feederTime.milliseconds() > servoTime * servoReturnMultiplier) {
            feedRing()
            ringsToShoot--
        }
        setFeederPosition(feederTime.milliseconds() < servoTime && rpmTarget != 0.0)
    }

    override fun updateExpansionHub() {
        currentVelocity = getMotorVelocity()
        currentRpm = currentVelocity * ticksToRpmMultiplier
        shooterRegulator.update(motorVelocityTarget)
    }

    override fun updateOther() {
    }

    private fun setFeederPosition(push: Boolean) {
        feederPositionSender.send(if (push) ShooterConfig.feederOpen else feederClose)
    }

    private fun setShooterSetings(Rpm: Double) {
        if (Rpm != rpmTarget) {
            rpmTarget = Rpm
            motorVelocityTarget = rpmTarget * 0.4
        }
    }

    private fun getMotorVelocity(): Double = shooterMotor.velocity //* 2.5

    var shootingMode: Shooter.ShooterMode
        get() = shooterMode
        set(mode) {
            //if (mode != Shooter.ShooterMode.OFF && shooterMode == Shooter.ShooterMode.OFF) rpmTime.reset()
            shooterMode = mode
            when (mode) {
                 Shooter.ShooterMode.HIGHGOAL -> setShooterSetings(ShooterConfig.highRpm)
                 Shooter.ShooterMode.POWERSHOT -> setShooterSetings(ShooterConfig.lowRpm)
                 Shooter.ShooterMode.OFF -> setShooterSetings(0.0)
            }
        }

    fun isCorrectRpm(error: Double = 30.0): Boolean {
        return if (encoderFailureMode) true else abs(rpmTarget - currentRpm) < error // abs(currentVelocity - rpmNow / 2.5) < error
    }

    fun feedRing() {
        //  ringsToShoot = 1
        feederTime.reset()
    }

    fun feedRings() {
        ringsToShoot = 4
    }
}