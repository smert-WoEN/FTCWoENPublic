package org.firstinspires.ftc.teamcode.robot.legacy

@Deprecated("")
class TwoWheelOdometry {/*: MultithreadRobotModule(), Odometry {
    private val odometryWheelDiameterCm = 4.8
    private val odometryCountsPerCM = 1440 / (odometryWheelDiameterCm * Math.PI)
    private val odometryCMPerCounts = odometryWheelDiameterCm * Math.PI / 1440
    private val odometerYcenterOffset =
        38.3633669516 * odometryCountsPerCM * cos(Math.toRadians(19.490773014)) / 2
    private val odometerXcenterOffset =
        36.8862986805 * odometryCountsPerCM * cos(Math.toRadians(67.021303041)) / 2
    private val yWheelPairRadiusCm = 18.425 //18.1275;
    private val radiansPerEncoderDifference = odometryCMPerCounts / (yWheelPairRadiusCm * 2)
    private val odometerY = 1
    private val odometerX = 2
    private var worldPosition = Pose2D()
    private var imuOffset = 0f
    private lateinit var imu: BNO055IMU
    private lateinit var expansionHub: ExpansionHubEx
    private lateinit var bulkData: RevBulkData
    private var looptime = ElapsedTime()
    private var lastAngle = 0.0
    private var yOld = 0
    private var xOld = 0
    private fun calculateHeading(L: Int, R: Int): Double {
        return MathUtil.angleWrap((L - R).toDouble() * radiansPerEncoderDifference - imuOffset)
    }

    private fun initIMU() {
        imu = imu2
        imu.initialize(BNO055IMU.Parameters())
        imuOffset = iMUheading.toFloat()
    }

    private var currentIMUHeading = 0.0
    private val iMUheading: Double = if (looptime.milliseconds() > 50) {
        looptime.reset()
        MathUtil.angleWrap(
            (-imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle - imuOffset).toDouble()
        ).also { lastAngle = it }
    } else lastAngle

    override fun updateControlHub() {
        currentIMUHeading = iMUheading
    }

    override fun updateExpansionHub() {
        calculatePosition(worldPosition)
    }

    override fun updateOther() {
    }

    @Synchronized
    fun calculatePosition(initialPose: Pose2D) {
        worldPosition = initialPose
        bulkData = expansionHub.bulkInputData
        val deltaWorldHeading = MathUtil.angleWrap(currentIMUHeading - worldPosition.heading)
        val deltaPosition = Vector2D(
            (bulkData.getMotorCurrentPosition(odometerX) - xOld).toDouble() - deltaWorldHeading * odometerXcenterOffset,
            (-bulkData.getMotorCurrentPosition(odometerY) - yOld).toDouble() - deltaWorldHeading * odometerYcenterOffset
        )
        worldPosition = worldPosition.plus(
            Pose2D(
                deltaPosition.rotatedCW(worldPosition.heading + deltaWorldHeading / 2),
                deltaWorldHeading
            )
        )
        yOld = -bulkData.getMotorCurrentPosition(odometerY)
        xOld = bulkData.getMotorCurrentPosition(odometerX)
    }

    override fun initialize() {
        assignNames()
        initIMU()
        delay(500.0)
        bulkData = expansionHub.bulkInputData
        yOld = -bulkData.getMotorCurrentPosition(odometerY)
        xOld = bulkData.getMotorCurrentPosition(odometerX)
    }

    private fun assignNames() {
        expansionHub = WoENHardware.expansionHub
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
        ) //.add(new Vector2D(0, 1).rotatedCW(worldPosition.heading));
        return Pose2D(poseTranslation, worldPosition.heading)
    }

    override fun setRobotCoordinates(coordinates: Pose2D) {
        imuOffset = MathUtil.angleWrap(
            calculateHeading(
                bulkData.getMotorCurrentPosition(odometerY),
                -bulkData.getMotorCurrentPosition(odometerY)
            ) + imuOffset - coordinates.heading
        ).toFloat()
        calculatePosition(
            Pose2D(
                coordinates.x * odometryCountsPerCM,
                coordinates.y * odometryCountsPerCM,
                coordinates.heading
            )
        )
    }

    override fun getRobotVelocity(): Vector3D {
        val angularVelocity = -imu.getAngularOrientation(
            AxesReference.INTRINSIC,
            AxesOrder.ZXY,
            AngleUnit.RADIANS
        ).firstAngle.toDouble()
        return Vector3D(
            Vector2D(
                (bulkData.getMotorCurrentPosition(odometerX)
                    .toDouble() - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                ((-bulkData.getMotorVelocity(odometerY)).toDouble() - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts
            ).rotatedCW(worldPosition.heading), angularVelocity
        )
    }*/
}
