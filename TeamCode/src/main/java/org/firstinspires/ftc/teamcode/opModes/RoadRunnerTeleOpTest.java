package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "RoadRunnerTest", group = "Real")
public class RoadRunnerTeleOpTest extends OpMode{
    private MecanumDrive driveBase;
    @Override
    public void init() {
        driveBase = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        double drive = .1;//-gamepad1.left_stick_y;
        double strafe = 0.1;//-gamepad1.left_stick_x;
        double turn = 0.1;//-gamepad1.right_stick_x;
        driveBase.setDrivePowers(new PoseVelocity2d( new Vector2d(drive, strafe), turn));

        driveBase.updatePoseEstimate();
        telemetry.addData("x", driveBase.pose.position.x);
        telemetry.addData("y", driveBase.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(driveBase.pose.heading.toDouble()));
        telemetry.update();
    }
}
