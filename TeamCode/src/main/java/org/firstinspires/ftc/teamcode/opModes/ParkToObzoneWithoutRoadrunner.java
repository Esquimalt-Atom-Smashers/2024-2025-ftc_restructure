package org.firstinspires.ftc.teamcode.opModes;


import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Autonomous", group = "Real")
public class ParkToObzoneWithoutRoadrunner extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, new Pose2d(0,0,0));
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void loop() {
//        robot.run();
//        robot.autoModeDrive(1,0,0,3);
    }
}