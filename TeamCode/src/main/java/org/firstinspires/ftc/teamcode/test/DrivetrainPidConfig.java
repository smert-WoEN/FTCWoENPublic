package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.misc.SinglePressButton;
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter;
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry;
import org.jetbrains.annotations.NotNull;

import java.util.List;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

@TeleOp
@Disabled
public class DrivetrainPidConfig extends LinearOpMode {

    @Config
    @Disabled
    public static class Constants {
        public static double achieveableMaxRPMFraction = 0.9;
        public static double achieveableMinRPMFraction = 0.05;
        public static double strafingMultiplier = 1 / 0.8;
        public static double rotationDecrepancy = 1.0;
        public static double kP = 1.5;
        public static double kD = 0;
        public static double kI = 0.15;
        public static double kF = 15;
    }

    DcMotorEx driveFrontLeft = null;
    DcMotorEx driveFrontRight = null;
    DcMotorEx driveRearLeft = null;
    DcMotorEx driveRearRight = null;


    public void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        driveFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveRearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveRearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void setRobotVelocity(double frontways, double sideways, double turn) {

        frontways *= forwardMultiplier;
        sideways *= sidewaysMultiplier;
        turn *= turnMultiplier;

        double FrontLeft = frontways + sideways + turn;
        double FrontRight = frontways - sideways - turn;
        double RearLeft = frontways - sideways + turn;
        double RearRight = frontways + sideways - turn;

        driveMotorPowers(FrontLeft, FrontRight, RearLeft, RearRight);
    }

    public void driveMotorPowers(double frontLeft, double frontRight, double rearLeft, double rearRight) {

        double maxabs = max(max(abs(frontLeft), abs(frontRight)), max(abs(rearLeft), abs(rearRight)));
        if (maxabs > maxMotorSpeed) {
            maxabs = maxabs / maxMotorSpeed;
            frontLeft = (frontLeft / maxabs);
            frontRight = (frontRight / maxabs);
            rearLeft = (rearLeft / maxabs);
            rearRight = (rearRight / maxabs);
        }

        powerFrontLeft = limitSpeed(frontLeft);
        powerFrontRight = limitSpeed(frontRight);
        powerRearLeft = limitSpeed(rearLeft);
        powerRearRight = limitSpeed(rearRight);
    }

    private double limitSpeed(double speed) {
        return clip(abs(speed), minMotorSpeed, maxMotorSpeed) * signum(speed);
    }

    private static final double wheelRadius = 9.8 / 2;
    private static final Vector2D wheelCenterOffset = new Vector2D(18.05253, 15.20000);
    private static final double forwardMultiplier = 1 / wheelRadius;
    private static double sidewaysMultiplier = forwardMultiplier * Constants.strafingMultiplier;
    private static double turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * Constants.rotationDecrepancy / wheelRadius;

    private void setMotorConfiguration(double achieveableMaxRPMFraction, double tickPerRev, double gearing, double maxRPM) {
        setMotorConfiguration(driveFrontLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveFrontRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveRearRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
    }

    private void setMotorConfiguration(@NotNull DcMotorEx dcMotor, double achieveableMaxRPMFraction, double tickPerRev, double gearing, double maxRPM) {
        MotorConfigurationType motorConfigurationType = dcMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(achieveableMaxRPMFraction);
        motorConfigurationType.setTicksPerRev(tickPerRev);
        motorConfigurationType.setGearing(gearing);
        motorConfigurationType.setMaxRPM(maxRPM);
        dcMotor.setMotorType(motorConfigurationType);
    }

    public Vector3D getMaxVelocity() {
        return new Vector3D(maxMotorSpeed / forwardMultiplier, maxMotorSpeed / forwardMultiplier, maxMotorSpeed / turnMultiplier);
    }

    private static final double tickPerRev = 480;
    private static final double gearing = 20;
    private static final double maxRPM = 300;
    private static final double theoreticalMaxSpeed = (maxRPM / 60) * Math.PI * 2;
    private static double maxMotorSpeed = Constants.achieveableMaxRPMFraction * theoreticalMaxSpeed;
    private static double minMotorSpeed = Constants.achieveableMinRPMFraction * theoreticalMaxSpeed;


    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerRearLeft = 0;
    private double powerRearRight = 0;

    private final double maxAcceleration = theoreticalMaxSpeed / 0.25;
    private final motorAccelerationLimiter mFLProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveFrontLeft.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    private final motorAccelerationLimiter mFRProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveFrontRight.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    private final motorAccelerationLimiter mRLProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveRearLeft.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    private final motorAccelerationLimiter mRRProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveRearRight.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);

    static final ThreeWheelOdometry odometry = new ThreeWheelOdometry();
    FtcDashboard dashboard;
    List<LynxModule> allHubs;

    @Override
    public void runOpMode(){
        waitForStart();
        odometry.initialize(this);
        dashboard = FtcDashboard.getInstance();
        driveFrontLeft = hardwareMap.get(DcMotorEx.class, "driveFrontLeft");
        driveFrontRight = hardwareMap.get(DcMotorEx.class, "driveFrontRight");
        driveRearLeft = hardwareMap.get(DcMotorEx.class, "driveRearLeft");
        driveRearRight = hardwareMap.get(DcMotorEx.class, "driveRearRight");
        driveFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotorEx.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorEx.Direction.REVERSE);
        driveFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveRearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveRearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveRearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        setPIDFCoefficients(new PIDFCoefficients(Constants.kP, Constants.kD, Constants.kI, Constants.kF));
        SinglePressButton sineResetter = new SinglePressButton(() -> gamepad1.b);
        ElapsedTime sineWaveTimer = new ElapsedTime();
        telemetry.setMsTransmissionInterval(40);
        for (LynxModule module : allHubs)
            module.clearBulkCache();
        odometry.updateAll();
        while (opModeIsActive()) {
            for (LynxModule module : allHubs)
                module.clearBulkCache();
            // odometry.update();
            Vector3D targetVelocity = new Vector3D(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x)
                    .times(getMaxVelocity());
            if (sineResetter.get())
                sineWaveTimer.reset();
            if (gamepad1.b)
                targetVelocity = new Vector3D(0, Math.sin(sineWaveTimer.seconds() * Math.PI / 3), 0).times(getMaxVelocity());
            telemetry.addData("targetX", targetVelocity.x);
            telemetry.addData("targety", targetVelocity.y);
            telemetry.addData("targetz", targetVelocity.z);
            setRobotVelocity(targetVelocity.y, targetVelocity.x, targetVelocity.z);
            Vector3D expectedVelocity = calculateEncoderDelta(powerFrontLeft, powerFrontRight, powerRearLeft, powerRearRight);
            telemetry.addData("commandX", expectedVelocity.x);
            telemetry.addData("commandY", expectedVelocity.y);
            telemetry.addData("commandZ", expectedVelocity.z);
            mFLProfiler.setVelocity(powerFrontLeft);
            mFRProfiler.setVelocity(powerFrontRight);
            mRLProfiler.setVelocity(powerRearLeft);
            mRRProfiler.setVelocity(powerRearRight);
            Vector3D wheelVelocity = calculateEncoderDelta(driveFrontLeft.getVelocity(AngleUnit.RADIANS), driveFrontRight.getVelocity(AngleUnit.RADIANS), driveRearLeft.getVelocity(AngleUnit.RADIANS), driveRearRight.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("wheelX", wheelVelocity.x);
            telemetry.addData("wheelY", wheelVelocity.y);
            telemetry.addData("wheelZ", wheelVelocity.z);
            if (gamepad1.back) {
                setPIDFCoefficients(new PIDFCoefficients(Constants.kP, Constants.kD, Constants.kI, Constants.kF));
                setMotorConfiguration(Constants.achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
                maxMotorSpeed = Constants.achieveableMaxRPMFraction * theoreticalMaxSpeed;
                minMotorSpeed = Constants.achieveableMinRPMFraction * theoreticalMaxSpeed;
                sidewaysMultiplier = forwardMultiplier * Constants.strafingMultiplier;
                turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * Constants.rotationDecrepancy / wheelRadius;
            }
            Vector3D velocity = odometry.getRobotVelocity();
            telemetry.addData("VelX", velocity.x);
            telemetry.addData("VelY", velocity.y);
            telemetry.addData("VelZ", velocity.z);
            telemetry.update();
        }
    }

    public Vector3D calculateEncoderDelta(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        return new Vector3D((frontLeft - frontRight - rearLeft - frontRight) / (4 * sidewaysMultiplier), (frontLeft + frontRight + rearLeft + rearRight) / (4 * forwardMultiplier), (frontLeft - frontRight + rearLeft - rearRight) / (4 * turnMultiplier));
    }

}
