package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name = "BasicTeleOp", group = "Real")
public class BasicTeleOp extends OpMode{
    private IMU imu;
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    private Servo servo;

    private boolean fieldCentric = true;
    private double servoPose;

    public void init(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(0);
        servoPose = 0;

        telemetry.addData("servo pos", servoPose);
    }

    public void loop(){
        drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

        if(gamepad1.y) {
            // move to 0 degrees.
            servo.setPosition(0);
            servoPose = 0;
            telemetry.addData("servo pos", servoPose);
        } else if (gamepad1.x || gamepad1.b) {
            // move to 90 degrees.
            servo.setPosition(0.5);
            servoPose = 0.5;
            telemetry.addData("servo pos", servoPose);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            servo.setPosition(1);
            servoPose = 1;
            telemetry.addData("servo pos", servoPose);
        }

        if (gamepad1.back) resetGyro();
        if (gamepad1.start) fieldCentric = !fieldCentric;

        telemetry.addData("heading", getHeading());
        telemetry.addData("fieldCentric", fieldCentric);
        telemetry.update();
    }

    private void drive(double forward, double strafe, double turn){

        if (fieldCentric){
            double gyroRadians = Math.toRadians(-getHeading());
            double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

            frontLeftMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe + turn), -1, 1) );
            frontRightMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe - turn), -1, 1) );
            backLeftMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe + turn), -1, 1) );
            backRightMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe - turn), -1, 1) );

        }else{
            frontLeftMotor.setPower(scaleInput(forward - strafe + turn));
            frontRightMotor.setPower(scaleInput(forward - strafe - turn));
            backLeftMotor.setPower(scaleInput(forward + strafe + turn));
            backRightMotor.setPower(scaleInput(forward + strafe - turn));
        }
    }


    private double scaleInput(double input){
        return Range.clip(input,-1,1);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void resetGyro(){
        imu.resetYaw();
    }

}