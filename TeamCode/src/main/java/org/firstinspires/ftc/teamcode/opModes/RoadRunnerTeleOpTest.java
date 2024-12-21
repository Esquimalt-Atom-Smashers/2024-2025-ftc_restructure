package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.libs.roadrunner.Mecanum20025;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name = "RoadRunnerTest", group = "Real")
public class RoadRunnerTeleOpTest extends OpMode{
    private Mecanum20025 driveBase;
    @Override
    public void init() {
        driveBase = new Mecanum20025(this.hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        driveBase.setDrivePowers(new PoseVelocity2d( new Vector2d(drive, strafe), turn));

        telemetry.addData("Now at",  "%7d :%7d",
                driveBase.updatePoseEstimate(),
                rightDrive.getCurrentPosition());
        telemetry.update();
    }
}
