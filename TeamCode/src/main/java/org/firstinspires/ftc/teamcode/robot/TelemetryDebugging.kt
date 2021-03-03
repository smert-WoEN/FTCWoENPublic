package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode
import org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import java.lang.Exception
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.cos
import kotlin.math.sin

class TelemetryDebugging : MultithreadRobotModule() {
    private val measurementTime = ElapsedTime()
    lateinit var dashboard: FtcDashboard
    lateinit var telemetry: Telemetry
    private lateinit var dashboardPacket: TelemetryPacket

    @Volatile
    var loopCount = AtomicInteger(0)

    @Volatile
    var controlHubLoopCount = AtomicInteger(0)

    @Volatile
    var expansionHubLooploopCount = AtomicInteger(0)

    @Config
    internal object TelemetryConfig {
        @JvmField
        var dashboardTelemetry = true
        @JvmField
        var refreshTimeMs = 50
    }

    override fun setOpMode(opMode: LinearOpMode) {
        super.setOpMode(opMode)
        dashboard = FtcDashboard.getInstance()
        telemetry = opMode.telemetry
    }

    private fun createDashboardRectangle(position: Pose2D, color: String) {
        val by = -position.x / 2.54
        val bx = position.y / 2.54
        val l = robotSideLength / (2.54 * 2)
        val l2 = l * 435.55 / 444
        val bxPoints = doubleArrayOf(l, -l, -l, l)
        val byPoints = doubleArrayOf(l2, l2, -l2, -l2)
        rotatePoints(bxPoints, byPoints, -position.heading)
        for (i in 0..3) {
            bxPoints[i] += bx
            byPoints[i] += by
        }
        dashboardPacket.fieldOverlay()
            .setStroke("cyan")
            .setStrokeWidth(2)
            .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2)
        dashboardPacket.fieldOverlay()
            .setStroke("orange")
            .setStrokeWidth(1)
            .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2)
        dashboardPacket.fieldOverlay()
            .setStroke(color)
            .setStrokeWidth(1)
            .strokePolygon(bxPoints, byPoints)
    }

    private var updaterIsActive = false
    private val updateTelemetry = Runnable {
        try {
            updaterIsActive = true
            while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
                if (measurementTime.milliseconds() > TelemetryConfig.refreshTimeMs) {
                    val loopFrequency = loopCount.getAndSet(0) / measurementTime.seconds()
                    val controlHubLoopFrequency =
                        controlHubLoopCount.getAndSet(0) / measurementTime.seconds()
                    val expansionHubloopFrequency =
                        expansionHubLooploopCount.getAndSet(0) / measurementTime.seconds()
                    measurementTime.reset()
                    telemetry.addData(
                        "Status",
                        "Running " + String.format(
                            "%.1f s @ %.1f Hz",
                            runTime.seconds(),
                            loopFrequency
                        )
                    )
                    //  telemetry.addData("CH Loop frequency", "$controlHubLoopFrequency Hz")
                    //  telemetry.addData("EH Loop frequency", "$expansionHubloopFrequency Hz")
                    val robotPosition = odometry.robotCoordinates
                    // telemetry.addLine("Odometry encoders").addData("odYL", WoENHardware.odometerYL.getCurrentPosition()).addData("odYR", WoENHardware.odometerYR.getCurrentPosition()).addData("odX", WoENHardware.odometerX.getCurrentPosition());
                    //telemetry.addLine("Robot position ").addData("Y", robotPosition.y).addData("X", robotPosition.x).addData("Head", Math.toDegrees(robotPosition.heading));
                    //   Vector3D velocity = odometry.getRobotVelocity();
                    //     telemetry.addLine("Robot velocity ").addData("Y", velocity.y).addData("X", velocity.x).addData("Head", Math.toDegrees(velocity.z));
                    telemetry.addLine("Shooter ").addData("Mode", shooter.shootingMode).addData(
                        "Current",
                        if (shooter.encoderFailureMode) "Encoder Fail" else shooter.currentRpm
                    ).addData("Target", shooter.rpmTarget)
                    //telemetry.addData("conpower", conveyor.conveyorPower);
                    //    telemetry.addLine("headings").addData("Encoder",Math.toDegrees(odometry.getEncoderHeading())).addData("IMU1",Math.toDegrees(odometry.getIMUheading_1())).addData("IMU2",Math.toDegrees(odometry.getIMUheading_2()));
                    if (TelemetryConfig.dashboardTelemetry) {
                        dashboardPacket = TelemetryPacket()
                        createDashboardRectangle(robotPosition, "black")
                        if (movement.pathFollowerIsActive()) createDashboardRectangle(
                            movement.currentTarget,
                            "green"
                        )
                        dashboardPacket.put("Loop frequency", loopFrequency)
                        // dashboardPacket.put("CH Loop frequency", controlHubLoopFrequency)
                        // dashboardPacket.put("EH Loop frequency", expansionHubloopFrequency)
                        // dashboardPacket.put("Flywhel RPM",shooter.currentRpm);
                        //  dashboardPacket.put("Flywhel target",shooter.rpmTarget);
                        dashboardPacket.put("Status", "Running " + runTime.seconds())
                        dashboard.sendTelemetryPacket(dashboardPacket)
                    }


                    //telemetry.addData("OpenCV stack size", openCVNode.getStackSize());
                    telemetry.update()
                } else Thread.yield()
            }
            updaterIsActive = false
        } catch (e: Exception) {
            telemetry.clear()
            telemetry.addData("Telemetry Error", e.message)
            telemetry.update()
        }
    }
    private var telemetryUpdater = Thread(updateTelemetry)
    override fun initialize() {
        measurementTime.reset()
        loopCount.getAndSet(0)
        //  telemetry = dashboard.getTelemetry();
        //  telemetry = new MultipleTelemetry(opMode.telemetry,dashboard.getTelemetry());
        telemetry.msTransmissionInterval = TelemetryConfig.refreshTimeMs
        //dashboard.startCameraStream(openCVNode.webcam,5.0);
    }

    override fun start() {
        telemetryUpdater.interrupt()
        telemetryUpdater = Thread(updateTelemetry)
        telemetryUpdater.start()
    }

    override fun updateControlHub() {
      //  controlHubLoopCount.getAndIncrement()
    }

    override fun updateExpansionHub() {
       // expansionHubLooploopCount.getAndIncrement()
    }

    override fun updateOther() {
        loopCount.getAndIncrement()
    }

    private var robotSideLength = 44.4
    private fun rotatePoints(xPoints: DoubleArray, yPoints: DoubleArray, angle: Double) {
        for (i in xPoints.indices) {
            val x = xPoints[i]
            val y = yPoints[i]
            xPoints[i] = x * cos(angle) - y * sin(angle)
            yPoints[i] = x * sin(angle) + y * cos(angle)
        }
    }
}