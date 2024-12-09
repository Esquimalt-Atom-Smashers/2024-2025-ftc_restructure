package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.libs.roadrunner.RoadrunnerConstants.DEADZONE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.libs.roadrunner.Mecanum20025;
import org.firstinspires.ftc.libs.roadrunner.RoadrunnerConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

public class DriveSubsystem extends SubsystemBase {
    private final HardwareMap hardwareMap;
    private final OpMode opMode;
    private final Telemetry telemetry;
    Mecanum20025 roadrunnerDrive;

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    public final DcMotorEx backLeftMotor;
    public final DcMotorEx backRightMotor;

    private final BHI260IMU imu;

    private double speedMultiplier = 1.0;
    private boolean fieldCentric = true;

    private ElapsedTime autoTimer = new ElapsedTime();
    private Pose2d updatedPose;
    private GoForNetZone goForNetZone;
    private TelemetryPacket telemetryPacket;

    public DriveSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        roadrunnerDrive = new Mecanum20025(opMode.hardwareMap, new Pose2d(-3, 1.5, Math.toRadians(90)));

        imu = hardwareMap.get(BHI260IMU.class, RoadrunnerConstants.IMU_NAME);
        imu.initialize(RoadrunnerConstants.IMU_PARAMETERS);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, RoadrunnerConstants.FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, RoadrunnerConstants.FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, RoadrunnerConstants.BACK_LEFT_MOTOR_NAME);
        backRightMotor = hardwareMap.get(DcMotorEx.class, RoadrunnerConstants.BACK_RIGHT_MOTOR_NAME);

        frontLeftMotor.setDirection(RoadrunnerConstants.FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(RoadrunnerConstants.FRONT_RIGHT_MOTOR_DIRECTION);
        backLeftMotor.setDirection(RoadrunnerConstants.BACK_LEFT_MOTOR_DIRECTION);
        backRightMotor.setDirection(RoadrunnerConstants.BACK_RIGHT_MOTOR_DIRECTION);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetGyro();
        resetEncoders();
    }

    public void drive(double forward, double strafe, double turn) {
        if(fieldCentric) driveFieldCentric(forward, strafe, turn);
        else driveRobotCentric(forward, strafe, turn);
        roadrunnerDrive.updatePoseEstimate();
        updatedPose = new Pose2d(roadrunnerDrive.pose.position.x, roadrunnerDrive.pose.position.y, roadrunnerDrive.pose.heading.toDouble());
    }

    private void driveFieldCentric(double forward, double strafe, double turn) {
        forward = GamepadUtils.deadzone(forward, DEADZONE);
        strafe = GamepadUtils.deadzone(strafe, DEADZONE);
        turn = GamepadUtils.deadzone(turn, DEADZONE);

        double gyroRadians = Math.toRadians(-getHeading());
        double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
        double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

        frontLeftMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        forward = GamepadUtils.deadzone(forward, DEADZONE);
        strafe = GamepadUtils.deadzone(strafe, DEADZONE);
        turn = GamepadUtils.deadzone(turn, DEADZONE);

        frontLeftMotor.setPower(Range.clip((forward + strafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((forward - strafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((forward - strafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((forward + strafe - turn), -1, 1) * speedMultiplier);
    }

    public void goForNetZone(){
        goForNetZone = new GoForNetZone();
        telemetryPacket = new TelemetryPacket();
        goForNetZone.run(telemetryPacket);
    }

    public void stopRoadrunner(){
        goForNetZone.cancelAbruptly();
    }

    public class GoForNetZone implements Action {
        private boolean cancelled = false;
        private Action action;

        public GoForNetZone() {
            TrajectoryActionBuilder goForNetZone = roadrunnerDrive.actionBuilder(updatedPose)
                    .splineToConstantHeading(new Vector2d(-2.9 * RoadrunnerConstants.INCH_TO_TILE ,2.9 * RoadrunnerConstants.INCH_TO_TILE),Math.toRadians(0));

            action = goForNetZone.build();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (cancelled) return false;
            return action.run(telemetryPacket);
        }

        public void cancelAbruptly() {
            cancelled = true;
        }
    }

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Range.clip(multiplier, 0, 1);
    }

    public void resetGyro() {
        imu.resetYaw();
    }

    public void setFieldCentricOnOff(){
        fieldCentric = !fieldCentric;
    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public boolean getIsFieldCentric() {
        return fieldCentric;
    }

    public void autoModeDrive(double forward, double strafe, double rotate, double duration){
        this.drive(forward, strafe, rotate);
        autoTimer.reset();

        if( autoTimer.seconds() == duration) {
            this.drive(0, 0, 0);
        }
    }
}

