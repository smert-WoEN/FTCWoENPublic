package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.math.MathUtil
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.antiWindupFraction_angle
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.antiWindupFraction_distance
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kD_angle
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kD_distance
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kI_angle
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kI_distance
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kP_angle
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kP_distance
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.minErrorAngleDefault
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.minErrorDistanceDefault
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MotionTask
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Odometry
import java.util.*
import kotlin.math.abs
import kotlin.math.sign

class Movement(private val odometry: Odometry, private val drivetrain: Drivetrain) : MultithreadedRobotModule() {
    @Config
    internal object MovementConfig {
        @JvmField var lookaheadRadius = 45.72
        @JvmField var kP_distance = 4.5
        @JvmField var kD_distance = 1.0
        @JvmField var kI_distance = 1.5
        @JvmField var kP_angle = 4.5
        @JvmField var kD_angle = .0
        @JvmField var kI_angle = 1.5
        @JvmField var antiWindupFraction_distance = .09
        @JvmField var antiWindupFraction_angle = .09
        @JvmField var minErrorDistanceDefault = 1.5
        @JvmField var minErrorAngleDefault = Math.toRadians(0.400)
    }

    val kP: Vector3D
    get() = Vector3D(kP_distance, kP_distance, kP_angle)

    val kI: Vector3D
        get() = Vector3D(kI_distance, kI_distance, kI_angle)

    val kD: Vector3D
        get() = Vector3D(kD_distance, kD_distance, kD_angle)

    // private val minErrorDistanceDefault = 1.0
    //private val minErrorAngleDefault =
    private var maxLinearVelocityFraction = 1.0
    private var maxAngularVelocityFraction = 1.0
    private var minErrorDistanceCurrent = minErrorDistanceDefault
    private var minErrorAngleCurrent = minErrorAngleDefault
    var nTargetPoint = 1
    var pathToFollow = ArrayList<MotionTask>()
    var bPathFollowerEnabled = false
    var bPathFollowingFinished = false
    var doActiveBraking = false
    var previousTarget = Pose2D()
    var previousError = Pose2D()
    var integralError = Vector3D()
    var previousVelocity = Vector3D()
    var requestedVelocityPercent = Vector3D(0.0, 0.0, 0.0)
    private var actionOnCompletionExecutor = Thread()
    private val pathFollowingTimer = ElapsedTime()


    override fun initialize() {
        start()
    }


    /**
     * Enable holding position by encoders even when target position is reached
     *
     * @param doActiveBraking if robot should brake with encoders
     */
    fun setActiveBraking(doActiveBraking: Boolean) {
        this.doActiveBraking = doActiveBraking
    }

    override fun start() {
        requestedVelocityPercent = Vector3D(0.0, 0.0, 0.0)
        nTargetPoint = 1
        bPathFollowerEnabled = false
        bPathFollowingFinished = false
        doActiveBraking = false
        pathToFollow = ArrayList()
        pathToFollow.add(0, MotionTask(odometry.robotCoordinates))
    }

    override fun updateControlHub() {
    }

    override fun updateExpansionHub() {
    }

    override fun updateOther() {
        bPathFollowingFinished = nTargetPoint >= pathToFollow.size
        if (pathFollowerIsActive() && requestedVelocityPercent.radius() < 0.005) {
            if (pathFollowingTimer.seconds() > 4) nTargetPoint++ else {
                val currentTarget = removeNaN(pathToFollow[nTargetPoint], odometry.robotCoordinates)
                val previousTarget = removeNaN(pathToFollow[nTargetPoint - 1], odometry.robotCoordinates)
                if (WoENrobot.movement.movePurePursuit(previousTarget, currentTarget, MovementConfig.lookaheadRadius)) {
                    if (WoENrobot.movement.moveLinear(currentTarget)) {
                        pathFollowingTimer.reset()
                        if (actionOnCompletionExecutor.state == Thread.State.NEW) actionOnCompletionExecutor.start() else if (actionOnCompletionExecutor.state == Thread.State.TERMINATED) {
                            nTargetPoint++
                            if (nTargetPoint < pathToFollow.size) actionOnCompletionExecutor = Thread(
                                 pathToFollow[nTargetPoint].actionOnCompletion)
                        }
                    }
                }
            }
        } else if (requestedVelocityPercent.radius() > 0.005) {
            if (pathFollowerIsActive()) stopPathFollowing()
            drivetrain.targetVelocity = requestedVelocityPercent * drivetrain.maxVelocity
           // if (pathToFollow.size > 0 && doActiveBraking) pathToFollow[pathToFollow.size - 1] = MotionTask(odometry.robotCoordinates)
        } else {
            if (pathToFollow.size > 0 && doActiveBraking) moveLinear(pathToFollow[pathToFollow.size - 1])
            else drivetrain.targetVelocity = Vector3D(.0, .0, .0)
        }
    }

    val currentTarget: Pose2D
        get() {
            return try {
                removeNaN(pathToFollow[nTargetPoint], odometry.robotCoordinates)
            } catch (e: IndexOutOfBoundsException) {
                Pose2D()
            }
        }


    /**
     * Checks whether path follower was disabled or have finished its job
     *
     * @return whether the path follower is active
     */
    fun pathFollowerIsActive(): Boolean {
        return bPathFollowerEnabled && !bPathFollowingFinished
    }

    /**
     * Legacy synchronous go-to-point (with custom speeds and tolerances)
     *
     * @param target                  Point to approach
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     * @param distanceTolerance       Minimum distance error
     * @param angularTolerance        Minimum angular error
     */
    fun pos(target: Pose2D, linearVelocityFraction: Double = 1.0, angularVelocityFraction: Double = 1.0, distanceTolerance: Double = minErrorDistanceDefault, angularTolerance: Double = minErrorAngleDefault) {
        followPath(MotionTask(target), linearVelocityFraction, angularVelocityFraction, distanceTolerance, angularTolerance)
        while (pathFollowerIsActive() && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    /**
     * Give path follower a task to go to point (with custom speeds and tolerances)
     *
     * @param motionTask              Points (motion tasks)
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     * @param distanceTolerance       Minimum distance error
     * @param angularTolerance        Minimum angular error
     */

    fun followPath(motionTask: MotionTask, linearVelocityFraction: Double = 1.0, angularVelocityFraction: Double = 1.0, distanceTolerance: Double = minErrorDistanceDefault, angularTolerance: Double = minErrorAngleDefault) {
        followPath(ArrayList(listOf(motionTask)), linearVelocityFraction, angularVelocityFraction, distanceTolerance, angularTolerance)
    }

    /**
     * Give path follower a task to follow an array of points (with custom speeds)
     *
     * @param pathToFollow            Array of points (motion tasks)
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     * @param distanceTolerance       Minimum distance error
     * @param angularTolerance        Minimum angular error
     */
    fun followPath(pathToFollow: ArrayList<MotionTask>, linearVelocityFraction: Double = 1.0, angularVelocityFraction: Double = 1.0, distanceTolerance: Double = minErrorDistanceDefault, angularTolerance: Double = minErrorAngleDefault) {
        minErrorDistanceCurrent = distanceTolerance
        minErrorAngleCurrent = angularTolerance
        setMaxLinearVelocityFraction(linearVelocityFraction)
        setMaxAngleVelocityFraction(angularVelocityFraction)
        bPathFollowerEnabled = false
        Thread.yield()
        this.pathToFollow = pathToFollow
        pathToFollow.add(0, MotionTask(odometry.robotCoordinates))
        Thread.yield()
        nTargetPoint = 1
        bPathFollowerEnabled = true
        bPathFollowingFinished = false
        actionOnCompletionExecutor = Thread(pathToFollow[nTargetPoint].actionOnCompletion)
        integralError = Vector3D()
        pathFollowingTimer.reset()
    }

    /**
     * Disable path follower (and start receiving teleop instructions)
     */
    fun stopPathFollowing() {
        bPathFollowerEnabled = false
    }

    fun setMaxLinearVelocityFraction(maxLinearVelocityFraction: Double) {
        this.maxLinearVelocityFraction = maxLinearVelocityFraction
    }

    fun setMinLinearVelocityFraction(minLinearVelocityFraction: Double) {
        antiWindupFraction_distance = minLinearVelocityFraction
    }

    fun setMaxAngleVelocityFraction(maxAngleVelocityFraction: Double) {
        this.maxAngularVelocityFraction = maxAngleVelocityFraction
    }

    fun setMinAngleVelocityFraction(minAngleVelocityFraction: Double) {
        antiWindupFraction_angle = minAngleVelocityFraction
    }

    private val moveControllerTimer = ElapsedTime()

    /**
     * Move to point Linearly
     *
     * @param target Pose Target point
     * @param velocity Movement velocity
     * @return Whether robot is within minimum error tolerance
     */
    private fun moveLinear(target: Pose2D, velocity: Vector2D? = null): Boolean {
        val error = getError(target)
        val currentVelocity = odometry.robotVelocity
        val diffError = (currentVelocity + previousVelocity) * -0.5
        previousVelocity = currentVelocity
        if (target == previousTarget && velocity == null) {
            integralError += Vector3D((error.x + previousError.x) * 0.5, (error.y + previousError.y) * 0.5,
                                      MathUtil.angleAverage(error.heading, previousError.heading)) * moveControllerTimer.seconds()
            /*integralError = Vector3D(Range.clip(abs(integralError.x), 0.0,
                                                MovementConfig.antiWindupFraction_distance * MovementConfig.kI_distance * drivetrain.maxVelocity.x) * sign(
                 integralError.x), Range.clip(abs(integralError.y), 0.0,
                                              MovementConfig.antiWindupFraction_distance * MovementConfig.kI_distance * drivetrain.maxVelocity.y) * sign(
                 integralError.y), Range.clip(abs(integralError.z), 0.0,
                                              MovementConfig.antiWindupFraction_angle * MovementConfig.kI_distance * drivetrain.maxVelocity.z) * sign(

                 integralError.z))*/
            integralError = integralError.clampAbs(drivetrain.maxVelocity * Vector3D(antiWindupFraction_distance, antiWindupFraction_distance, antiWindupFraction_angle) * kI)
        } else integralError = Vector3D()
        moveControllerTimer.reset()
        previousError = error
        previousTarget = target
        integralError = Vector3D(Vector2D(.0,integralError.radius()).rotatedCW(error.heading),integralError.z )
        val control =
            (Vector3D(error) * kP).clampAbs(drivetrain.maxVelocity) +
            (integralError * kI).clampAbs(drivetrain.maxVelocity) +
            (diffError * kD).clampAbs(drivetrain.maxVelocity)
        if (velocity != null) holonomicMoveFC(Vector3D((error as Vector2D).normalize() * velocity.radius(), control.z))
        else holonomicMoveFC(control)
        return abs(error.heading) < minErrorAngleCurrent && error.radius() < minErrorDistanceCurrent
        //drivetrain.setRobotVelocity(0, 0, 0);
    }

    /**
     * Calculates positional error
     *
     * @param target Pose we want to be in
     * @return Difference between current position and given target
     */
    fun getError(target: Pose2D): Pose2D {
        return removeNaN(target, odometry.robotCoordinates) - odometry.robotCoordinates
    }

    /**
     * Move to point using Pure Pursuit
     *
     * @param originPoint     Trajectory starting point
     * @param targetPoint     Trajectory ending point
     * @param lookaheadRadius Pure pursuit lookahead radius
     * @return If robotPosition is within lookahead radius of the target
     */
    private fun movePurePursuit(originPoint: Vector2D, targetPoint: Pose2D, lookaheadRadius: Double): Boolean {
        val robotPosition = odometry.robotCoordinates
        if (originPoint.x == targetPoint.x && originPoint.y == targetPoint.y) return true
        val a = originPoint.y - targetPoint.y
        val b = targetPoint.x - originPoint.x
        val c = originPoint.x * targetPoint.y - originPoint.y * targetPoint.x
        var angle = MathUtil.angleWrap((targetPoint - originPoint).aCot())
        val lookAheadPoint = Vector2D((b * (b * robotPosition.x - a * robotPosition.y) - a * c) / (a * a + b * b),
                                      (a * (-b * robotPosition.x + a * robotPosition.y) - b * c) / (a * a + b * b)) + Vector2D(0.0,
                                                                                                                               lookaheadRadius).rotatedCW(
             angle)
        if (abs(MathUtil.angleWrap(
                  angle - if ((originPoint - robotPosition).radius() > lookaheadRadius) targetPoint.heading else robotPosition.heading)) > (if ((originPoint - robotPosition).radius() > lookaheadRadius * 2) Math.PI / 2 else Math.PI / 4)) angle = MathUtil.angleWrap(
             angle + if ((targetPoint - robotPosition).radius() > lookaheadRadius * 2 || abs(
                       MathUtil.angleWrap(targetPoint.heading - angle + Math.PI)) < Math.PI / 4) Math.PI else Math.PI / 2 * sign(
                  MathUtil.angleWrap(targetPoint.heading - angle)))
        if ((targetPoint - originPoint).radius() <= (lookAheadPoint - originPoint).radius()) {
            if (getError(targetPoint).radius() < lookaheadRadius) return true
            moveLinear(Pose2D(targetPoint, angle))
        } else moveLinear(Pose2D(lookAheadPoint, angle), Vector3D(getError(targetPoint))*kP-odometry.robotVelocity*kD)
        return false
    }

    /**
     * Receive teleop movement instructions
     *
     * @param x    x velocity (-1.0; 1.0)
     * @param y    x velocity (-1.0; 1.0)
     * @param turn angular velocity (-1.0; 1.0)
     */
    fun humanSetVelocity(x: Double, y: Double, turn: Double) {
        requestedVelocityPercent = Vector3D(x, y, turn)
    }

    /**
     * Field-centric move
     *
     * @param velocityCommand Robot velocities in cm/s
     */
    private fun holonomicMoveFC(velocityCommand: Vector3D) {
        var linearVelocity = Vector2D(velocityCommand.x, velocityCommand.y).rotatedCW(-odometry.robotCoordinates.heading)
        linearVelocity = linearVelocity.normalize() * Range.clip(linearVelocity.radius(), 0.0,
                                                                 drivetrain.maxVelocity.y * maxLinearVelocityFraction)
        val angularVelocity = Range.clip(abs(velocityCommand.z), 0.0, drivetrain.maxVelocity.z * maxAngularVelocityFraction) * sign(
             velocityCommand.z)
        drivetrain.targetVelocity = Vector3D(linearVelocity, angularVelocity)
    }
    /* public void holonomicMovePolar(double heading, double speed, double turn) {
        turn = Range.clip(turn, -1.0, 1.0);
        speed = Range.clip(speed, -1.0, 1.0);
        double frontways = speed * cos(heading);
        double sideways = speed * sin(heading);

        drivetrain.setRobotVelocity(frontways, sideways, turn);
    }*/

    /**
     * Replaces NaNs in Pose2D with odometry coordinates
     *
     * @param pose2D Pose to remove NaNs from
     * @param odometryPose odometry Pose to replace NaNs with
     * @return Pose without NaNs
     */
    private fun removeNaN(pose2D: Pose2D, odometryPose: Pose2D): Pose2D {
        val newPose = pose2D.clone()
        if (pose2D.x.isNaN()) newPose.x = odometryPose.x
        if (pose2D.y.isNaN()) newPose.y = odometryPose.y
        if (pose2D.heading.isNaN()) newPose.heading = odometryPose.heading
        return newPose
    }
}