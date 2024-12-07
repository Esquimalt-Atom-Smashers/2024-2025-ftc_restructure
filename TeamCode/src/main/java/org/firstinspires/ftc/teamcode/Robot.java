package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.libs.roadrunner.Mecanum20025;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {
    private final OpMode opMode;

    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    private final Mecanum20025 driveSubsystem;

    public Robot(OpMode opMode, Pose2d pose2D) {
        this.opMode = opMode;

        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        driveSubsystem = new Mecanum20025(opMode.hardwareMap, pose2D);
    }

    public void configureTeleOpBindings() {
        /* Driver controls:
         *   Forward -> left y axis
         *   Strafe -> left x axis
         *   Turn -> right x axis
         *
         *   Reduce Speed -> right bumper
         *   Reset Gyro -> back button
         *   Enable/Disable Field Centric -> b
         */
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        //Driver Controls
/*
        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger speedVariationTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        speedVariationTrigger.whenActive(() -> driveSubsystem.setSpeedMultiplier(0.2));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        /*
        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());
        */

    }}
    /*
    public void run() {
        CommandScheduler.getInstance().run();
        opMode.telemetry.addData("Linear Slide Position:", linearSlideSubsystem.getCurrentPosition());
        opMode.telemetry.addData("Elbow Current Position:", elbowSubsystem.getCurrentPosition());
        opMode.telemetry.addData("Wrist Current Position:", intakeSubsystem.getCurrentPosition());
        opMode.telemetry.update();
    }
}
*/