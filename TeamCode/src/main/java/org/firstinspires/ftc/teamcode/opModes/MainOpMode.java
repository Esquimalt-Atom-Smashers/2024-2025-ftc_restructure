package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.libs.roadrunner.RoadrunnerConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

@TeleOp(name = "TeleOp", group = "Real")
public class MainOpMode extends OpMode {
    Robot robot;

    public final double DEADZONE = 0.1;

    @Override
    public void init() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(this);
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;

        driveSub
    }

    public class DriveSubsystem extends SubsystemBase {
        private final HardwareMap hardwareMap;
        private final OpMode opMode;
        private final Telemetry telemetry;

        private final DcMotorEx frontLeftMotor;
        private final DcMotorEx frontRightMotor;
        public final DcMotorEx backLeftMotor;
        public final DcMotorEx backRightMotor;

        private final BHI260IMU imu;

        private double speedMultiplier = 1.0;
        private boolean fieldCentric = true;

        public DriveSubsystem(OpMode opMode) {
            this.opMode = opMode;
            this.hardwareMap = opMode.hardwareMap;
            this.telemetry = opMode.telemetry;

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
    }
}