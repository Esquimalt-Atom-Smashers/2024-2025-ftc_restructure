package org.firstinspires.ftc.libs.roadrunner;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RoadrunnerConstants {
    public final static String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
    public final static String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
    public final static String BACK_LEFT_MOTOR_NAME = "rearLeftMotor";
    public final static String BACK_RIGHT_MOTOR_NAME = "rearRightMotor";

    public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final DcMotorSimple.Direction LEFT_ENCODER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_ENCODER_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction CENTER_ENCODER_DIRECTION = DcMotorSimple.Direction. REVERSE;

    public static final String IMU_NAME = "imu";
    public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            )
    );

    public static final RevHubOrientationOnRobot IMU_PARAMETERS_ROADRUNNER = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    public static final double DEADZONE = 0.1;

    public  static double INCH_TO_TILE = 24;
}
