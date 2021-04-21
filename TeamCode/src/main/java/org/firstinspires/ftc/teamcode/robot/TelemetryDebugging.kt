package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.robot.TelemetryDebugging.TelemetryConfig.publishVelocityInfo
import org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.robot.WoENrobot.voltageSupplier
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.cos
import kotlin.math.sin

class TelemetryDebugging : MultithreadedRobotModule() {
    private val measurementTime = ElapsedTime()
    lateinit var dashboard: FtcDashboard
    lateinit var telemetry: Telemetry
    private lateinit var dashboardPacket: TelemetryPacket

    @Volatile var loopCount = AtomicInteger(0)

    @Volatile var controlHubLoopCount = AtomicInteger(0)

    @Volatile var expansionHubLooploopCount = AtomicInteger(0)

    @Config
    internal object TelemetryConfig {
        @JvmField var dashboardTelemetry = true
        @JvmField var refreshTimeMs = 50
        @JvmField var publishVelocityInfo = false
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
        dashboardPacket.fieldOverlay().setStroke("#00EEFF").setStrokeWidth(2) //cyan
            .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2)
        dashboardPacket.fieldOverlay().setStroke("#FFA700").setStrokeWidth(1) //orange
            .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2)
        dashboardPacket.fieldOverlay().setStroke(color).setStrokeWidth(1).strokePolygon(bxPoints, byPoints)
    }

    private var updaterIsActive = false
    private val updateTelemetry = Runnable {
        try {
            updaterIsActive = true
            while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
                if (measurementTime.milliseconds() > TelemetryConfig.refreshTimeMs) {
                    val loopFrequency = loopCount.getAndSet(0) / measurementTime.seconds()
                    val controlHubLoopFrequency = controlHubLoopCount.getAndSet(0) / measurementTime.seconds()
                    val expansionHubloopFrequency = expansionHubLooploopCount.getAndSet(0) / measurementTime.seconds()
                    measurementTime.reset()
                    telemetry.addData("Status", "Running " + String.format("%.1f s @ %.1f Hz", runTime.seconds(), loopFrequency))
                    telemetry.addData("CH Loop frequency", "$controlHubLoopFrequency Hz")
                    telemetry.addData("EH Loop frequency", "$expansionHubloopFrequency Hz")
                    val robotPosition = odometry.robotCoordinates
                    // telemetry.addLine("Odometry encoders").addData("odYL", WoENHardware.odometerYL.getCurrentPosition()).addData("odYR", WoENHardware.odometerYR.getCurrentPosition()).addData("odX", WoENHardware.odometerX.getCurrentPosition());
                    telemetry.addLine("Robot position ").addData("Y", robotPosition.y).addData("X", robotPosition.x).addData("Head", Math.toDegrees(robotPosition.heading))
                    //   Vector3D velocity = odometry.getRobotVelocity();
                    //     telemetry.addLine("Robot velocity ").addData("Y", velocity.y).addData("X", velocity.x).addData("Head", Math.toDegrees(velocity.z));
                    telemetry.addLine("Shooter ").addData("Mode", shooter.shootingMode)
                        .addData("Current", if (shooter.encoderFailureMode) "Encoder Fail" else shooter.currentRpm)
                        .addData("Target", shooter.rpmTarget)
                    //telemetry.addData("conpower", conveyor.conveyorPower);
                    //    telemetry.addLine("headings").addData("Encoder",Math.toDegrees(odometry.getEncoderHeading())).addData("IMU1",Math.toDegrees(odometry.getIMUheading_1())).addData("IMU2",Math.toDegrees(odometry.getIMUheading_2()));
                    if (TelemetryConfig.dashboardTelemetry) {
                        dashboardPacket = TelemetryPacket()
                        createDashboardRectangle(robotPosition, "black")
                        if (movement.pathFollowerIsActive()) createDashboardRectangle(movement.currentTarget, "#66CC66")
                        dashboardPacket.put("Status", "Running " + runTime.seconds())
                        dashboardPacket.put("Loop frequency", loopFrequency)
                        dashboardPacket.put("CH Loop frequency", controlHubLoopFrequency)
                        dashboardPacket.put("EH Loop frequency", expansionHubloopFrequency)
                        dashboardPacket.put("Battery Voltage", voltageSupplier.voltage)
                        dashboardPacket.put("Head", Math.toDegrees(robotPosition.heading))
                        dashboardPacket.put("Flywheel RPM", shooter.currentRpm)
                        dashboardPacket.put("Flywheel target", shooter.rpmTarget)
                        dashboardPacket.put("", odometry.status)
                        if(publishVelocityInfo) {
                            val targetVelocity = drivetrain.targetVelocity
                            val drivetrainVelocity = drivetrain.robotVelocity//.rotatedCW(robotPosition.heading)
                            val odometryVelocity = odometry.robotVelocity.rotatedCW(-robotPosition.heading)
                            dashboardPacket.put("targetX", targetVelocity.x)
                            dashboardPacket.put("targetY", targetVelocity.y)
                            dashboardPacket.put("targetH", targetVelocity.z)
                            dashboardPacket.put("driveX", drivetrainVelocity.x)
                            dashboardPacket.put("driveY", drivetrainVelocity.y)
                            dashboardPacket.put("driveH", drivetrainVelocity.z)
                            dashboardPacket.put("odoX", odometryVelocity.x)
                            dashboardPacket.put("odoY", odometryVelocity.y)
                            dashboardPacket.put("odoH", odometryVelocity.z)
                        }
                        dashboard.sendTelemetryPacket(dashboardPacket)
                    }


                    //telemetry.addData("OpenCV stack size", openCVNode.getStackSize());
                    telemetry.update()
                } else Thread.yield()
            }
            updaterIsActive = false
        } catch (e: Exception) {
            telemetry.clear()
            telemetry.addData("Telemetry Error", e.toString())
            dashboardPacket = TelemetryPacket()
            dashboardPacket.put("Telemetry Error", e.toString())
            dashboard.sendTelemetryPacket(dashboardPacket)
            e.printStackTrace()
            telemetry.update()
        }
    }
    private var telemetryUpdater = Thread(updateTelemetry)
    override fun initialize() {
        dashboard = FtcDashboard.getInstance()
        telemetry = opMode.telemetry
        measurementTime.reset()
        loopCount.getAndSet(0)
        telemetry.msTransmissionInterval = TelemetryConfig.refreshTimeMs
        //dashboard.startCameraStream(openCVNode.webcam,5.0);
    }

    override fun start() {
        dashboard = FtcDashboard.getInstance()
        telemetry = opMode.telemetry
        telemetryUpdater.interrupt()
        telemetryUpdater = Thread(updateTelemetry)
        telemetryUpdater.start()
    }

    override fun updateControlHub() {
        controlHubLoopCount.getAndIncrement()
    }

    override fun updateExpansionHub() {
        expansionHubLooploopCount.getAndIncrement()
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