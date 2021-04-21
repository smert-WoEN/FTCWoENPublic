package org.firstinspires.ftc.teamcode.robot.simulation

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.math.MathUtil
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain
import org.firstinspires.ftc.teamcode.robot.VoltageSupplier
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Odometry
import kotlin.math.abs
import kotlin.math.sign

class FakeDrivetrainOdometry : MultithreadedRobotModule(), Drivetrain, Odometry {
    // private val maxVelocity = MecanumDrivetrain().maxVelocity
    private var started = false
    private var targetVelocityFC = Vector3D(0.0, 0.0, 0.0)
    override var robotVelocity: Vector3D = Vector3D()
    private var realVelocityFC = Vector3D(0.0, 0.0, 0.0)

    override val maxVelocity: Vector3D = MecanumDrivetrain(VoltageSupplier()).maxVelocity

    private val zLimiter = MotorAccelerationLimiter({ realVelocityFC.z = it }, maxVelocity.z / 0.35)
    private val yLimiter = MotorAccelerationLimiter({ realVelocityFC.y = it }, maxVelocity.y / 0.35)
    private val xLimiter = MotorAccelerationLimiter({ realVelocityFC.x = it }, maxVelocity.x / 0.35)
    private val updateTimer = ElapsedTime()

    override var robotCoordinates = Pose2D()

    override fun initialize() {
        targetVelocity = Vector3D(0.0, 0.0, 0.0)
        realVelocityFC = Vector3D(0.0, 0.0, 0.0)
        robotCoordinates = Pose2D(0.0, 0.0, 0.0)
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
        //realVelocityFC.x = targetVelocityFC.x
        //realVelocityFC.y = targetVelocityFC.y
        //realVelocityFC.z = targetVelocityFC.z
        zLimiter.setVelocity(targetVelocityFC.z)
        yLimiter.setVelocity(targetVelocityFC.y)
        xLimiter.setVelocity(targetVelocityFC.x)
        robotVelocity = realVelocityFC
        robotCoordinates.y += realVelocityFC.y * updateTimer.seconds()
        robotCoordinates.x += realVelocityFC.x * updateTimer.seconds()
        robotCoordinates.heading = MathUtil.angleWrap(this.robotCoordinates.heading + realVelocityFC.z * updateTimer.seconds())
        updateTimer.reset()
    }

    override var targetVelocity = Vector3D(0.0, 0.0, 0.0)
        set(value) {
            var frontWays = value.y
            var sideWays = value.x
            var turn = value.z
            if (abs(frontWays) > maxVelocity.y) frontWays = maxVelocity.y * sign(frontWays)
            if (abs(sideWays) > maxVelocity.x) sideWays = maxVelocity.x * sign(sideWays)
            if (abs(turn) > maxVelocity.z) turn = maxVelocity.z * sign(turn)
            field = Vector3D(sideWays, frontWays, turn)
            targetVelocityFC = Vector3D(Vector2D(sideWays, frontWays).rotatedCW(this.robotCoordinates.heading), turn)
        }


}