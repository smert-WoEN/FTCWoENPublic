package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxVoltageSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.ExpansionHubServo

object WoENHardware {

    lateinit var hardwareMap: HardwareMap

    lateinit var lynxModules: List<LynxModule>
    lateinit var controlHub: ExpansionHubEx
    lateinit var expansionHub: ExpansionHubEx

    lateinit var controlHubVoltageSensor: VoltageSensor
    lateinit var expansionHubVoltageSensor: VoltageSensor

    lateinit var driveFrontLeft: ExpansionHubMotor
    lateinit var driveFrontRight: ExpansionHubMotor
    lateinit var driveRearLeft: ExpansionHubMotor
    lateinit var driveRearRight: ExpansionHubMotor
    lateinit var odometerYL: ExpansionHubMotor
    lateinit var odometerYR: ExpansionHubMotor
    lateinit var odometerX: ExpansionHubMotor
    lateinit var conveyorMotor: ExpansionHubMotor
    lateinit var shooterMotor: ExpansionHubMotor
    lateinit var lever: ExpansionHubMotor
    lateinit var ledStrip1: ExpansionHubMotor
    lateinit var ledStrip2: ExpansionHubMotor

    lateinit var feeder: ExpansionHubServo
    lateinit var gripper: ExpansionHubServo
    lateinit var leverArm: ExpansionHubServo

    lateinit var ringDetector: DistanceSensor

    lateinit var controlHubIMU: BNO055IMU
    lateinit var imu2: BNO055IMU


    @JvmOverloads
    fun assignHardware(
        hwMap: HardwareMap = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().rootActivity).hardwareMap
    ) {

        hardwareMap = hwMap

        lynxModules = hardwareMap.getAll(LynxModule::class.java)

        controlHub = hardwareMap.get(ExpansionHubEx::class.java, "Control Hub")
        expansionHub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")

        controlHubVoltageSensor =
            LynxVoltageSensor(hardwareMap.appContext, controlHub.standardModule)
        expansionHubVoltageSensor =
            LynxVoltageSensor(hardwareMap.appContext, expansionHub.standardModule)

        driveFrontLeft =
            hardwareMap.get(DcMotorEx::class.java, "driveFrontLeft") as ExpansionHubMotor
        driveFrontRight =
            hardwareMap.get(DcMotorEx::class.java, "driveFrontRight") as ExpansionHubMotor
        driveRearLeft = hardwareMap.get(DcMotorEx::class.java, "driveRearLeft") as ExpansionHubMotor
        driveRearRight =
            hardwareMap.get(DcMotorEx::class.java, "driveRearRight") as ExpansionHubMotor
        odometerYL = hardwareMap.get(DcMotorEx::class.java, "odometerYL") as ExpansionHubMotor
        odometerYR = hardwareMap.get(DcMotorEx::class.java, "odometerYR") as ExpansionHubMotor
        odometerX = hardwareMap.get(DcMotorEx::class.java, "odometerXConveyor") as ExpansionHubMotor
        conveyorMotor =
            hardwareMap.get(DcMotorEx::class.java, "odometerXConveyor") as ExpansionHubMotor
        shooterMotor = hardwareMap.get(DcMotorEx::class.java, "shooterMotor") as ExpansionHubMotor
        lever = hardwareMap.get(DcMotorEx::class.java, "odometerYR") as ExpansionHubMotor
        ledStrip1 = hardwareMap.get(DcMotorEx::class.java, "odometerYR") as ExpansionHubMotor
        ledStrip2 = hardwareMap.get(DcMotorEx::class.java, "odometerYL") as ExpansionHubMotor

        feeder = hardwareMap.get(Servo::class.java, "feeder") as ExpansionHubServo
        gripper = hardwareMap.get(Servo::class.java, "wobbleGrabber") as ExpansionHubServo
        leverArm = hardwareMap.get(Servo::class.java, "angle") as ExpansionHubServo

        ringDetector = hardwareMap.get(DistanceSensor::class.java, "ringDetector")

        controlHubIMU = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu2 = hardwareMap.get(BNO055IMU::class.java, "imu 1")
    }
}