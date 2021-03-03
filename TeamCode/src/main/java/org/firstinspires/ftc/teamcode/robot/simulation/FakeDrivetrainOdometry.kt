package org.firstinspires.ftc.teamcode.robot.simulation

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.math.MathUtil
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Odometry
import kotlin.math.abs
import kotlin.math.sign

class FakeDrivetrainOdometry : MultithreadRobotModule(), Drivetrain, Odometry {
   // private val maxVelocity = MecanumDrivetrain().maxVelocity
    private var started = false
    private var targetVelocity = Vector3D(0.0, 0.0, 0.0)
    private var targetVelocityFC = Vector3D(0.0, 0.0, 0.0)
    private var realVelocityFC = Vector3D(0.0, 0.0, 0.0)
    private val zLimiter =
        motorAccelerationLimiter({realVelocityFC.z = it}, maxVelocity.z / 0.38)
    private val yLimiter =
        motorAccelerationLimiter({realVelocityFC.y = it}, maxVelocity.y / 0.38)
    private val xLimiter =
        motorAccelerationLimiter({realVelocityFC.x = it}, maxVelocity.x / 0.38)
    private var currentPosition = Pose2D(0.0, 0.0, 0.0)
    private val updateTimer = ElapsedTime()
    override fun initialize() {
        targetVelocity = Vector3D(0.0, 0.0, 0.0)
        realVelocityFC = Vector3D(0.0, 0.0, 0.0)
        currentPosition = Pose2D(0.0, 0.0, 0.0)
        updateTimer.reset()
    }

    override fun start() {
        targetVelocity = Vector3D(0.0, 0.0, 0.0)
        realVelocityFC = Vector3D(0.0, 0.0, 0.0)
        updateTimer.reset()
        started = false
    }

    override fun updateControlHub() {
    }

    override fun updateExpansionHub() {
    }

    override fun updateOther() {
        if (!started) {
            started = true
            updateTimer.reset()
        }
        // realVelocityFC.x = targetVelocityFC.x;
        zLimiter.setVelocity(targetVelocityFC.z)
        yLimiter.setVelocity(targetVelocityFC.y)
        xLimiter.setVelocity(targetVelocityFC.x)
        currentPosition.y += realVelocityFC.y * updateTimer.seconds()
        currentPosition.x += realVelocityFC.x * updateTimer.seconds()
        currentPosition.heading =
            MathUtil.angleWrap(currentPosition.heading + realVelocityFC.z * updateTimer.seconds())
        updateTimer.reset()
    }

    override fun setRobotVelocity(frontways: Double, sideways: Double, turn: Double) {
        var frontWays = frontways
        var sideWays = sideways
        var Turn = turn
        if (abs(frontWays) > maxVelocity.y) frontWays = maxVelocity.y * sign(frontWays)
        if (abs(sideWays) > maxVelocity.x) sideWays = maxVelocity.x * sign(sideWays)
        if (abs(Turn) > maxVelocity.z) Turn = maxVelocity.z * sign(Turn)
        targetVelocity = Vector3D(sideWays, frontWays, Turn)
        targetVelocityFC =
            Vector3D(Vector2D(sideWays, frontWays).rotatedCW(currentPosition.heading), Turn)
    }

    override val maxVelocity: Vector3D
            get() = MecanumDrivetrain().maxVelocity

    override fun getRobotCoordinates(): Pose2D {
        return currentPosition
    }

    override fun setRobotCoordinates(coordinates: Pose2D) {
        currentPosition = coordinates.clone()
    }

    override fun getRobotVelocity(): Vector3D {
        return realVelocityFC
    }
}