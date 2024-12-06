package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.libs.roadrunner.Mecanum20025;

public class DriveSubsystem extends Mecanum20025 {
    public DriveSubsystem(HardwareMap hardwareMap, Pose2d pose){
        super(hardwareMap, pose);

    }
}
