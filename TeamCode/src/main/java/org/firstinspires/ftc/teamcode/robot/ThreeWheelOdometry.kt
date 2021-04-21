package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.math.MathUtil
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.Encoder
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Odometry
import kotlin.math.cos
import kotlin.math.sin

class ThreeWheelOdometry : MultithreadedRobotModule(), Odometry {

    @Config
    internal object OdometryConfig {
        @JvmField var forwardMultiplier = 1.00
        @JvmField var headingMultiplier = 1.0213407782003108056291390728477
        @JvmField var doUseIMU = false
    }

    override val status
    get() = "ThreeWheelOdometry: pos{YL=$ylOld YR=$yrOld X=$xOld} vel{YL=$currentYLVelocity YR=$currentYRVelocity X=$currentXVelocity}"

    private val yWheelPairRadiusCm = 18.000
    override var robotCoordinates = Pose2D()
        @Synchronized set
    var startCoordinates = Pose2D()
        set(value) {
            robotCoordinates = value
            field = value
        }
    override var robotVelocity: Vector3D = Vector3D()

    private var angleOffset = 0.0
    private lateinit var imu1: BNO055IMU
    private lateinit var imu2: BNO055IMU
    private lateinit var odometerYL: Encoder
    private lateinit var odometerYR: Encoder
    private lateinit var odometerX: Encoder
    private val encoderCPR = 8192.0
    private var odometerWheelDiameterCm = OdometryConfig.forwardMultiplier * 4.8
    private var odometerCountsPerCM = encoderCPR / (odometerWheelDiameterCm * Math.PI)
    private var encoderCMPerCounts = odometerWheelDiameterCm * Math.PI / encoderCPR
    private var odometerXCenterOffset = -18.15937 * cos(Math.toRadians(84.9452)) //* odometerCountsPerCM?
    private var radiansPerEncoderDifference = OdometryConfig.headingMultiplier * (encoderCMPerCounts / (yWheelPairRadiusCm * 2.0))
    private var imuOffset1 = 0f
    private var imuOffset2 = 0f
    private var encoderHeadingCovariance = 0.0
    private var ylOld = 0
    private var yrOld = 0
    private var xOld = 0
    private val yWheelPairCenterOffset = Vector2D(0.0, 6.35375)
    private val imu1AccessTimer = ElapsedTime()
    private val imu2AccessTimer = ElapsedTime()

    private var doUseIMULocal = OdometryConfig.doUseIMU

    private fun calculateHeading(): Double = MathUtil.angleWrap(encoderDifferenceToAngle(odometerYL.currentPosition.toDouble(),
                                                                                         odometerYR.currentPosition.toDouble()) - angleOffset - encoderHeadingCovariance + startCoordinates.heading)

    private fun encoderTicksToDistance(ticks: Int) = ticks.toDouble() * encoderCMPerCounts

    private fun encoderDifferenceToAngle(L: Double, R: Double): Double = (L - R) * radiansPerEncoderDifference

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
        currentIMU1Heading = if (doUseIMULocal) MathUtil.angleWrap((-imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY,
                                                                                                AngleUnit.RADIANS).firstAngle - imuOffset1).toDouble()) else -0.0
    }

    private var currentIMU2Heading = 0.0
    private fun updateEHIMUHeading() {
        currentIMU2Heading = if (doUseIMULocal) MathUtil.angleWrap((-imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY,
                                                                                                AngleUnit.RADIANS).firstAngle - imuOffset2).toDouble()) else -0.0
    }

    override fun start() {
        if (doUseIMULocal) {
            imuOffset1 = -imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle
            imuOffset2 = -imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle
        }
    }

    override fun updateControlHub() {
        if (doUseIMULocal && imu1AccessTimer.seconds() > 1) {
            updateCHIMUHeading()
            val angleDivergence = MathUtil.angleWrap(encoderDifferenceToAngle(ylOld.toDouble(), yrOld.toDouble()) - currentIMU1Heading)
            encoderHeadingCovariance = angleDivergence * (1.0 / 2.0)
            imu1AccessTimer.reset()
        }
    }


    override fun updateExpansionHub() {
        calculatePosition()
        currentYLVelocity = odometerYL.correctedVelocity
        currentYRVelocity = odometerYR.correctedVelocity
        currentXVelocity = odometerX.correctedVelocity
        calculateVelocity()
    }

    override fun updateOther() {
    }

    private fun calculatePosition() {
        val currentYL = odometerYL.currentPosition
        val currentYR = odometerYR.currentPosition
        val currentX = odometerX.currentPosition
        val deltaHeading = MathUtil.angleWrap(calculateHeading() - robotCoordinates.heading)
        var deltaPosition = Vector2D(encoderTicksToDistance(currentX - xOld) - deltaHeading * odometerXCenterOffset,
                                     (encoderTicksToDistance(currentYL - ylOld) + encoderTicksToDistance(
                                         currentYR - yrOld)) * 0.5)
        if (deltaHeading != 0.0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
            val arcAngle = deltaHeading * 2
            val arcRadius = deltaPosition.radius() / arcAngle
            deltaPosition = Vector2D(arcRadius * (1 - cos(arcAngle)), arcRadius * sin(arcAngle)).rotatedCW(deltaPosition.aCot())
        }
        robotCoordinates = robotCoordinates.plus(Pose2D(deltaPosition.rotatedCW(robotCoordinates.heading), deltaHeading))
        ylOld = currentYL
        yrOld = currentYR
        xOld = currentX
    }

    private fun calculateVelocity() {
        val angularVelocity = encoderDifferenceToAngle(currentYLVelocity, currentYRVelocity)
        robotVelocity = Vector3D(encoderTicksToDistance((currentXVelocity.toInt())) - angularVelocity * odometerXCenterOffset,
                                          encoderTicksToDistance((currentYLVelocity + currentYRVelocity).toInt() / 2), angularVelocity).rotatedCW(
            robotCoordinates.heading)
    }

    override fun initialize() {
        doUseIMULocal = OdometryConfig.doUseIMU
        if (doUseIMULocal) initIMU()
        radiansPerEncoderDifference = OdometryConfig.headingMultiplier * (encoderCMPerCounts / (yWheelPairRadiusCm * 2.0))
        odometerWheelDiameterCm = OdometryConfig.forwardMultiplier * 4.8
        odometerCountsPerCM = encoderCPR / (odometerWheelDiameterCm * Math.PI)
        encoderCMPerCounts = odometerWheelDiameterCm * Math.PI / encoderCPR
        //odometerXCenterOffset = -21.7562349 * odometerCountsPerCM * cos(Math.toRadians(51.293002))
        WoENHardware.odometerYL.let {
            odometerYL = Encoder(it, Encoder.Direction.REVERSE)
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        WoENHardware.odometerYR.let {
            odometerYR = Encoder(it, Encoder.Direction.REVERSE)
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        WoENHardware.odometerX.let {
            odometerX = Encoder(it, Encoder.Direction.REVERSE)
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        ylOld = odometerYL.currentPosition
        yrOld = odometerYR.currentPosition
        xOld = odometerX.currentPosition
    }

    private var currentYLVelocity: Double = .0
    private var currentYRVelocity: Double = .0
    private var currentXVelocity: Double = .0
}