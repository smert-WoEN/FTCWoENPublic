package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.math.MathUtil
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.Encoder
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Odometry
import kotlin.math.cos
import kotlin.math.sin

class ThreeWheelOdometry : MultithreadRobotModule(), Odometry {
    private val yWheelPairRadiusCm = 18.000
    private var worldPosition = Pose2D()
    private var angleOffset = 0.0
    private lateinit var imu1: BNO055IMU
    private lateinit var imu2: BNO055IMU
    private lateinit var odometerYL: Encoder
    private lateinit var odometerYR: Encoder
    private lateinit var odometerX: Encoder
    private val endoderCPR = 8192.0
    private var odometryWheelDiameterCm = OdometryConfig.forwardMultiplier * 4.8
    private var odometryCountsPerCM = endoderCPR / (odometryWheelDiameterCm * Math.PI)
    private var odometryCMPerCounts = odometryWheelDiameterCm * Math.PI / endoderCPR
    private var odometerXcenterOffset =
        -18.15937 * odometryCountsPerCM * cos(Math.toRadians(84.9452))
    private var radiansPerEncoderDifference =
        OdometryConfig.headingMultiplier * (odometryCMPerCounts / (yWheelPairRadiusCm * 2.0))
    private var imuOffset1 = 0f
    private var imuOffset2 = 0f
    private var encoderHeadingCovariance = 0.0
    private var ylOld = 0
    private var yrOld = 0
    private var xOld = 0
    private val yWheelPairCenterOffset = Vector2D(0.0, 6.35375)
    private val imu1AccessTimer = ElapsedTime()
    private val imu2AccessTimer = ElapsedTime()

    @Config
    internal object OdometryConfig {
        @JvmField
        var forwardMultiplier = 1.00

        @JvmField
        var headingMultiplier = 1.0178671389975555

        @JvmField
        var doUseIMU = false
    }

    private var doUseIMULocal = OdometryConfig.doUseIMU
    private fun calculateHeading(): Double {
        return MathUtil.angleWrap(encoderHeading - angleOffset - encoderHeadingCovariance)
    }

    private val encoderHeading: Double
        get() = getEncoderHeading(
            odometerYL.currentPosition.toDouble(),
            odometerYR.currentPosition.toDouble()
        )

    private fun getEncoderHeading(L: Double, R: Double): Double {
        return (L - R) * radiansPerEncoderDifference
    }

    private fun initIMU() {
        imu1 = WoENHardware.controlHubIMU
        val parameters1 = BNO055IMU.Parameters()
        parameters1.accelRange = BNO055IMU.AccelRange.G2
        parameters1.gyroRange = BNO055IMU.GyroRange.DPS500
        parameters1.mode = BNO055IMU.SensorMode.IMU
        parameters1.calibrationDataFile = "BNO055IMUCalibration_1.json"
        imu1.initialize(parameters1)
        imu2 = WoENHardware.imu2
        val parameters2 = BNO055IMU.Parameters()
        parameters2.accelRange = BNO055IMU.AccelRange.G2
        parameters2.gyroRange = BNO055IMU.GyroRange.DPS500
        parameters2.mode = BNO055IMU.SensorMode.IMU
        parameters2.calibrationDataFile = "BNO055IMUCalibration_2.json"
        imu2.initialize(parameters2)
        encoderHeadingCovariance = 0.0
        imu1AccessTimer.reset()
        imu2AccessTimer.reset()
    }

    private var currentIMU1Heading = 0.0
    private fun updateCHIMUHeading() {
        currentIMU1Heading = if (doUseIMULocal) MathUtil.angleWrap(
            (-imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle - imuOffset1).toDouble()
        ) else -0.0
    }

    private var currentIMU2Heading = 0.0
    private fun updateEHIMUHeading() {
        currentIMU2Heading = if (doUseIMULocal) MathUtil.angleWrap(
            (-imu2.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle - imuOffset2).toDouble()
        ) else -0.0
    }

    override fun start() {
        if (doUseIMULocal) {
            imuOffset1 = -imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle
            imuOffset2 = -imu2.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle
        }
    }

    override fun updateControlHub() {
        if (doUseIMULocal && imu1AccessTimer.seconds() > 1) {
            updateCHIMUHeading()
            val angleDivergence = MathUtil.angleWrap(encoderHeading - currentIMU1Heading) //TODO
            encoderHeadingCovariance = angleDivergence * (1.0 / 2.0)
            imu1AccessTimer.reset()
        }
    }


    override fun updateExpansionHub() {

        currentYLVelocity = odometerYL.correctedVelocity
        currentYLVelocity = odometerYR.correctedVelocity
        currentYLVelocity = odometerX.correctedVelocity
        calculatePosition(worldPosition)
    }

    override fun updateOther() {
    }

    @Synchronized
    fun calculatePosition(initialPose: Pose2D) {
        worldPosition = initialPose
        val deltaWorldHeading = MathUtil.angleWrap(calculateHeading() - worldPosition.heading)
        var deltaPosition = Vector2D(
            (odometerX.currentPosition - xOld).toDouble() - deltaWorldHeading * odometerXcenterOffset,
            (odometerYL.currentPosition - ylOld + (odometerYR.currentPosition - yrOld)).toDouble() / 2
        )
        if (deltaWorldHeading != 0.0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
            val arcAngle = deltaWorldHeading * 2
            val arcRadius = deltaPosition.radius() / arcAngle
            deltaPosition = Vector2D(
                arcRadius * (1 - cos(arcAngle)),
                arcRadius * sin(arcAngle)
            ).rotatedCW(deltaPosition.acot())
        }
        worldPosition = worldPosition.plus(
            Pose2D(
                deltaPosition.rotatedCW(worldPosition.heading),
                deltaWorldHeading
            )
        )
        ylOld = odometerYL.currentPosition
        yrOld = odometerYR.currentPosition
        xOld = odometerX.currentPosition
    }

    override fun initialize() {
        doUseIMULocal = OdometryConfig.doUseIMU
        if (doUseIMULocal) initIMU()
        radiansPerEncoderDifference =
            OdometryConfig.headingMultiplier * (odometryCMPerCounts / (yWheelPairRadiusCm * 2.0))
        odometryWheelDiameterCm = OdometryConfig.forwardMultiplier * 4.8
        odometryCountsPerCM = endoderCPR / (odometryWheelDiameterCm * Math.PI)
        odometryCMPerCounts = odometryWheelDiameterCm * Math.PI / endoderCPR
        odometerXcenterOffset =
            -21.7562349 * odometryCountsPerCM * cos(Math.toRadians(51.293002))
        WoENHardware.odometerYL.let{
            odometerYL = Encoder(it, Encoder.Direction.REVERSE)
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        WoENHardware.odometerYR.let{
            odometerYR = Encoder(it, Encoder.Direction.REVERSE)
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        WoENHardware.odometerX.let{
            odometerX = Encoder(it, Encoder.Direction.REVERSE)
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        ylOld = odometerYL.currentPosition
        yrOld = odometerYR.currentPosition
        xOld = odometerX.currentPosition
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    override fun getRobotCoordinates(): Pose2D {
        val poseTranslation = Vector2D(
            worldPosition.x * odometryCMPerCounts,
            worldPosition.y * odometryCMPerCounts
        ).minus(yWheelPairCenterOffset.rotatedCW(worldPosition.heading))
        return Pose2D(poseTranslation, worldPosition.heading)
    }

    override fun setRobotCoordinates(coordinates: Pose2D) {
        ylOld = odometerYL.currentPosition
        yrOld = odometerYR.currentPosition
        xOld = odometerX.currentPosition
        angleOffset = MathUtil.angleWrap(calculateHeading() + angleOffset - coordinates.heading)
        calculatePosition(
            Pose2D(
                Vector2D(
                    coordinates.x * odometryCountsPerCM,
                    coordinates.y * odometryCountsPerCM
                ).plus(
                    yWheelPairCenterOffset.times(odometryCountsPerCM)
                        .rotatedCW(worldPosition.heading)
                ),
                coordinates.heading
            )
        )
    }

    private var currentYLVelocity: Double = .0
    private var currentYRVelocity: Double = .0
    private var currentXVelocity: Double = .0

    override fun getRobotVelocity(): Vector3D {
        val angularVelocity = getEncoderHeading(currentYLVelocity, currentYRVelocity)
        return Vector3D(
            Vector2D(
                (currentXVelocity - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                (currentYLVelocity + currentYRVelocity) * odometryCMPerCounts / 2
            ).rotatedCW(worldPosition.heading), angularVelocity
        )
    }
}