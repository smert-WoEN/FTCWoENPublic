package org.firstinspires.ftc.teamcode.robot.legacy

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.robot.WoENHardware
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Shooter
import org.openftc.revextensions2.ExpansionHubServo
import kotlin.math.abs

@Deprecated("")
class Shooter : MultithreadedRobotModule(), Shooter {
    private val rpmTime = ElapsedTime()
    private val feederTime = ElapsedTime()
    private val encoderFailureDetectionTime = ElapsedTime()


    @Config
    @Disabled
    internal object ShooterConfigOld {
        @JvmField var servoTime = 137.0
        @JvmField var servoReturnMultiplier = 2.6
        @JvmField var lowRpm = 3470.0
        @JvmField var highRpm = 4000.0
        @JvmField var timeRpm = 150.0
        @JvmField var feederClose = 0.225
        @JvmField var feederOpen = 0.48
        @JvmField var kP = 58.0
        @JvmField var kI = 0.001 //0.03
        @JvmField var kD = 0.05
        @JvmField var kF = 14.89
        @JvmField var kF_referenceVoltage = 12.485
    }

    private lateinit var shooterMotor: DcMotorEx
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var feeder: ExpansionHubServo
    private val shooterVelocitySender = CommandSender({ p: Double -> shooterMotor.velocity = p })
    private val feederPositionSender = CommandSender({ p: Double -> feeder.position = p })
    private var shooterMode = Shooter.ShooterMode.OFF
    private var ringsToShoot: Int = 0
    private var timeToAccelerateMs = 1.0
    private var accelerationIncrement = 1.0
    var rpmTarget = 6000.0
        private set
    private var motorVelocityTarget = 0.0
    var currentRpm = 0.0
    var encoderFailureMode = false
        private set

    override fun initialize() {
        voltageSensor = WoENHardware.expansionHubVoltageSensor
        shooterMotor = WoENHardware.shooterMotor
        val motorConfigurationType = shooterMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 0.896476253
        motorConfigurationType.ticksPerRev = 24.0
        motorConfigurationType.gearing = 1.0
        motorConfigurationType.maxRPM = 6000.0
        shooterMotor.motorType = motorConfigurationType
        try {
            shooterMotor.setVelocityPIDFCoefficients(ShooterConfigOld.kP, ShooterConfigOld.kI, ShooterConfigOld.kD,
                                                     ShooterConfigOld.kF * ShooterConfigOld.kF_referenceVoltage / voltageSensor.voltage)
        } catch (e: UnsupportedOperationException) {
            opMode.telemetry.addData("Shooter PIDF error ", e.message)
        }
        shooterMotor.direction = DcMotorSimple.Direction.FORWARD
        shooterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shootingMode = Shooter.ShooterMode.OFF
        initializedservo()
        feederTime.reset()
    }

    private fun initializedservo() {
        feeder = opMode.hardwareMap.get(Servo::class.java, "feeder") as ExpansionHubServo
        feeder.position = ShooterConfigOld.feederClose
    }

    override fun start() {
        feeder.position = ShooterConfigOld.feederClose
        shooterMotor.velocity = 0.0
        shootingMode = Shooter.ShooterMode.OFF
        ringsToShoot = 0
    }

    override fun updateControlHub() {
        if (ringsToShoot > 0 && feederTime.milliseconds() > ShooterConfigOld.servoTime * ShooterConfigOld.servoReturnMultiplier) {
            feedRing()
            ringsToShoot--
        }
        setFeederPosition(feederTime.milliseconds() < ShooterConfigOld.servoTime && motorVelocityTarget != 0.0)
    }

    override fun updateExpansionHub() {
        shooterVelocitySender.send(
             if (rpmTime.milliseconds() >= timeToAccelerateMs) motorVelocityTarget else rpmTime.milliseconds() * accelerationIncrement * motorVelocityTarget)
        if (encoderFailureDetectionTime.seconds() > 1) if (motorVelocityTarget == 0.0 || getMotorRpm() != 0.0) encoderFailureDetectionTime.reset()
        if (motorVelocityTarget != 0.0 && ringsToShoot == 0) updatePIDFCoeffs(encoderFailureDetectionTime.seconds() > 3)
        currentRpm = getMotorRpm()
    }

    override fun updateOther() {
    }

    private val pidfUpdateTimer = ElapsedTime()
    private fun updatePIDFCoeffs(encoderFailureMode: Boolean) {
        if (encoderFailureMode != this.encoderFailureMode || pidfUpdateTimer.seconds() > 3.0) {
            this.encoderFailureMode = encoderFailureMode
            pidfUpdateTimer.reset()
            try {
                if (this.encoderFailureMode) shooterMotor.setVelocityPIDFCoefficients(0.0, 0.0, 0.0,
                                                                                      ShooterConfigOld.kF * ShooterConfigOld.kF_referenceVoltage / voltageSensor.voltage) else shooterMotor.setVelocityPIDFCoefficients(
                    ShooterConfigOld.kP, ShooterConfigOld.kI, ShooterConfigOld.kD,
                    ShooterConfigOld.kF * ShooterConfigOld.kF_referenceVoltage / voltageSensor.voltage)
            } catch (ignored: UnsupportedOperationException) {
            }
        }
    }

    private fun setFeederPosition(push: Boolean) {
        feederPositionSender.send(if (push) ShooterConfigOld.feederOpen else ShooterConfigOld.feederClose)
    }

    private fun setShootersetings(Rpm: Double, time: Double) {
        if (Rpm != rpmTarget || time != timeToAccelerateMs) {
            rpmTarget = Rpm
            if (time != 0.0) timeToAccelerateMs = abs(time)
            accelerationIncrement = rpmTarget / timeToAccelerateMs / 6000
            motorVelocityTarget = rpmTarget * 0.4
        }
    }

    private fun getMotorRpm(): Double = shooterMotor.velocity * 2.5

    var shootingMode: Shooter.ShooterMode
        get() = shooterMode
        set(mode) {
            if (mode != Shooter.ShooterMode.OFF && shooterMode == Shooter.ShooterMode.OFF) rpmTime.reset()
            shooterMode = mode
            when (mode) {
                 Shooter.ShooterMode.HIGHGOAL -> setShootersetings(ShooterConfigOld.highRpm, ShooterConfigOld.timeRpm)
                 Shooter.ShooterMode.POWERSHOT -> setShootersetings(ShooterConfigOld.lowRpm, ShooterConfigOld.timeRpm)
                 Shooter.ShooterMode.OFF -> setShootersetings(0.0, ShooterConfigOld.timeRpm)
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
        ringsToShoot = 3
    }
}