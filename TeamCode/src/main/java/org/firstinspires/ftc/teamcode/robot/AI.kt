package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor

class AI : MultithreadedRobotModule() {
    private val aiTime = ElapsedTime()
    private val aiTimeControlHub = ElapsedTime()
    private val aiTimeExpansionHub = ElapsedTime()
    private val timeDiagnostic = 1000.0
    private val timeWait = 500.0
    private lateinit var odometerYL: ExpansionHubMotor
    private lateinit var odometerYR: ExpansionHubMotor
    private lateinit var conveyorm: ExpansionHubMotor
    private lateinit var shooterMotor: ExpansionHubMotor
    private lateinit var driveFrontLeft: ExpansionHubMotor
    private lateinit var driveFrontRight: ExpansionHubMotor
    private lateinit var driveRearLeft: ExpansionHubMotor
    private lateinit var driveRearRight: ExpansionHubMotor

    override fun initialize() {
        odometerYL = WoENHardware.odometerYL
        odometerYR = WoENHardware.odometerYR
        conveyorm = WoENHardware.conveyorMotor
        shooterMotor = WoENHardware.shooterMotor
        driveFrontLeft = WoENHardware.driveFrontLeft
        driveFrontRight = WoENHardware.driveFrontRight
        driveRearLeft = WoENHardware.driveRearLeft
        driveRearRight = WoENHardware.driveRearRight
        aiTime.reset()
        aiTimeExpansionHub.reset()
        aiTimeControlHub.reset()
    }


    override fun updateOther() {
    }

    override fun updateControlHub() {
        /*  if (aiTimeControlHub.milliseconds() > timeDiagnostic) {
              aiTimeControlHub.reset()
              if (tempMotor(driveFrontLeft) || tempMotor(driveFrontRight) || tempMotor(driveRearLeft) || tempMotor(driveRearRight)) {
                  opMode.telemetry.addData("Warning!", "overHeadControlHub")
              }
          }*/
    }

    override fun updateExpansionHub() {
        /*if (aiTimeExpansionHub.milliseconds() > timeDiagnostic){
            aiTimeExpansionHub.reset()
            if (tempMotor(conveyorm) || tempMotor(ShooterMotor) || tempMotor(OdometerYL) || tempMotor(OdometerYR)){
                opMode.telemetry.addData("Warning!", "overHeadExpansionHub")
            }
        }*/
    }

    private fun tempMotor(motor: ExpansionHubMotor): Boolean {
        return motor.isBridgeOverTemp
    }

    fun diagnosticServo(servo: Servo, startPos: Double, endPos: Double) {
        aiTime.reset()
        servo.position = startPos
        while (opMode.opModeIsActive() && aiTime.milliseconds() < timeDiagnostic / 2) {
            Thread.yield()
        }
        servo.position = endPos
        while (opMode.opModeIsActive() && aiTime.milliseconds() < timeDiagnostic) {
            Thread.yield()
        }
    }

    fun diagnosticRange(sensor: DistanceSensor): Boolean {
        return !sensor.getDistance(DistanceUnit.CM).isNaN()
    }

    fun getAmpsHub(Hub: ExpansionHubEx): Double {
        return Hub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS)
    }

    fun getGPIOAmpsHub(Hub: ExpansionHubEx): Double {
        return Hub.getGpioBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS)
    }

    fun getI2CAmpsHub(Hub: ExpansionHubEx): Double {
        return Hub.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS)
    }

    fun get5vHub(Hub: ExpansionHubEx): Double {
        return Hub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)
    }

    fun get12vHub(Hub: ExpansionHubEx): Double {
        return Hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)
    }

    fun diagnosticMotor(motor: DcMotorEx): Boolean {
        aiTime.reset()
        motor.power = 1.0
        do {
            if (motor.getCurrent(CurrentUnit.AMPS) > 0.5 && aiTime.milliseconds() > timeWait) {
                motor.power = 0.0
                return true
            }
        } while (opMode.opModeIsActive() && aiTime.milliseconds() < timeDiagnostic)
        motor.power = 0.0
        return false
    }

}