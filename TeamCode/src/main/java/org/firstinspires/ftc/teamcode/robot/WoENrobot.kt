package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.WoENHardware.assignHardware
import org.firstinspires.ftc.teamcode.robot.WoENHardware.lynxModules
import org.firstinspires.ftc.teamcode.robot.simulation.OpenCVNodePhonecam
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.openftc.revextensions2.ExpansionHubEx
import java.lang.Exception
import java.util.*

object WoENrobot {
    val wobbleManipulator = ServoWobbleManipulator()
    val openCVNode = OpenCVNodeWebcam()
    val conveyor = Conveyor()
    val shooter = Shooter()
    val telemetryDebugging = TelemetryDebugging()
    val ai = AI()
    //val odometry = FakeRobot();
    //val drivetrain = odometry;
    val odometry = ThreeWheelOdometry()
    val drivetrain = MecanumDrivetrain()
    val movement = Movement(odometry, drivetrain)
    private val activeRobotModules = arrayOf(
        odometry,
        movement,
        drivetrain,
        wobbleManipulator,
        conveyor,
        shooter,
        telemetryDebugging,
        ai
    ) //


    lateinit var opMode: LinearOpMode
    var robotIsInitialized = false
    val runTime = ElapsedTime()
    @Volatile
    var controlHubSpinCompleted = false

    @Volatile
    var expansionHubSpinCompleted = false

    @Volatile
    var spinCompleted = false
    lateinit var controlHub: ExpansionHubEx
    lateinit var expansionHub: ExpansionHubEx
    private lateinit var allHubs: List<LynxModule>
    var updateControlHub = Runnable {
        try {
            controlHub.standardModule.bulkCachingMode = BulkCachingMode.MANUAL
            while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
                controlHub.standardModule.clearBulkCache()
                Arrays.stream(activeRobotModules)
                    .forEach { obj: MultithreadRobotModule -> obj.updateControlHub() }
                controlHubSpinCompleted = true
            }

        }catch (e: InterruptedException) {}
    }
    private var controlHubUpdater = Thread(updateControlHub)
    var updateExpansionHub = Runnable {
        try {
            expansionHub.standardModule.bulkCachingMode = BulkCachingMode.MANUAL
            while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
                expansionHub.standardModule.clearBulkCache()
                Arrays.stream(activeRobotModules)
                    .forEach { obj: MultithreadRobotModule -> obj.updateExpansionHub() }
                expansionHubSpinCompleted = true
            }
        }catch (e: InterruptedException) {}
    }
    private var expansionHubUpdater = Thread(updateExpansionHub)
    var updateOther = Runnable {
        try {
            while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
                while ((!controlHubSpinCompleted || !expansionHubSpinCompleted) && opMode.opModeIsActive()) {
                    Thread.yield()
                }
                Arrays.stream(activeRobotModules)
                    .forEach { obj: MultithreadRobotModule -> obj.updateOther() }
                controlHubSpinCompleted = false
                expansionHubSpinCompleted = false
                spinCompleted = true
            }
        }catch (e: InterruptedException) {}
    }
    private var otherUpdater = Thread(updateOther)
    var updateRegulators = Runnable {
        try {
            setBulkCachingMode(BulkCachingMode.MANUAL)
            while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
                clearBulkCaches()
                Arrays.stream(activeRobotModules)
                    .forEach { obj: MultithreadRobotModule -> obj.updateAll() }
                spinCompleted = true
            }
        } catch (e: Exception) {
            opMode.requestOpModeStop()
        }
    }
    private var regulatorUpdater = Thread(updateRegulators)
    fun fullInitWithCV(OpMode: LinearOpMode) {
        openCVNode.initialize(OpMode)
        forceInitRobot(OpMode)
    }

    @Synchronized
    private fun spinHubs() {
        controlHubSpinCompleted = false
        expansionHubSpinCompleted = false
        while ((!controlHubSpinCompleted || !expansionHubSpinCompleted) && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    fun spinOnce() {
        spinCompleted = false
        while (!spinCompleted && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    fun delay(delay_ms: Double) {
        val timer = ElapsedTime()
        timer.reset()
        while (timer.milliseconds() < delay_ms && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    fun startRobot() {
        setLedColors(255, 166, 0)
        opMode.waitForStart()
        if (opMode.isStopRequested) return
        runTime.reset()
        Arrays.stream(activeRobotModules).forEach { obj: MultithreadRobotModule -> obj.start() }
        regulatorUpdater.start();
        //controlHubUpdater.start()
        //expansionHubUpdater.start()
        //otherUpdater.start()
        setLedColors(0, 237, 255)
        opMode.telemetry.addData("Status", "Running")
        opMode.telemetry.update()
    }

    fun initRobot(OpMode: LinearOpMode) {
        if (!robotIsInitialized) {
            forceInitRobot(OpMode)
            opMode.telemetry.addData("Status", "Initialization successful")
            opMode.telemetry.update()
        } else {
            opMode = OpMode
            Arrays.stream(activeRobotModules)
                .forEach { robotModule: MultithreadRobotModule -> robotModule.setOpMode(opMode) }
            if (regulatorUpdater.state != Thread.State.NEW) {
                regulatorUpdater.interrupt()
                regulatorUpdater = Thread(updateRegulators)
            }
            controlHubUpdater.interrupt()
            controlHubUpdater = Thread(updateControlHub)
            expansionHubUpdater.interrupt()
            expansionHubUpdater = Thread(updateExpansionHub)
            otherUpdater.interrupt()
            otherUpdater = Thread(updateOther)
            opMode.telemetry.addData("Status", "Already initialized, ready")
            opMode.telemetry.update()
        }
    }

    fun forceInitRobot(OpMode: LinearOpMode) {
        opMode = OpMode
        opMode.telemetry.addData("Status", "Initializing...")
        opMode.telemetry.update()
        assignHardware(opMode.hardwareMap)
        controlHub = WoENHardware.controlHub
        expansionHub = WoENHardware.expansionHub
        allHubs = lynxModules
        setBulkCachingMode(BulkCachingMode.MANUAL)
        Arrays.stream(activeRobotModules)
            .forEach { robotModule: MultithreadRobotModule -> robotModule.initialize(opMode) }
        regulatorUpdater.interrupt()
        regulatorUpdater = Thread(updateRegulators)
        controlHubUpdater.interrupt()
        controlHubUpdater = Thread(updateControlHub)
        expansionHubUpdater.interrupt()
        expansionHubUpdater = Thread(updateExpansionHub)
        otherUpdater.interrupt()
        otherUpdater = Thread(updateOther)
        //stopAllMotors();
        robotIsInitialized = true
        opMode.telemetry.addData("Status", "Force initialized")
        opMode.telemetry.update()
    }

    fun simpleInit(OpMode: LinearOpMode) {
        initRobot(OpMode)
        startRobot()
    }

    fun setLedColors(r: Int, g: Int, b: Int) {
        controlHub.setLedColor(r, g, b)
        expansionHub.setLedColor(r, g, b)
    }

    private fun setBulkCachingMode(mode: BulkCachingMode?) {
        for (module in allHubs) module.bulkCachingMode = mode
    }

    private fun clearBulkCaches() {
        for (module in allHubs) module.clearBulkCache()
    }

    fun fullInit(OpMode: LinearOpMode) {
        forceInitRobot(OpMode)
    }
}