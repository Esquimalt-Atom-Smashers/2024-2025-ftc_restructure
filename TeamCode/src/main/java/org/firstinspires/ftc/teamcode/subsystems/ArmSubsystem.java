package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arm Subsystem", group = "Real")
public class ArmSubsystem extends OpMode {

    private DcMotor armExtenderMotor = null;
    private double minExtension;
    private double maxExtension;

    public void init() {
        armExtenderMotor = hardwareMap.get(DcMotorEx.class, "armExtender");

        armExtenderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armExtenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        double armExtensionInput = gamepad2.right_stick_y;

        armExtenderMotor.setPower(Range.clip(armExtensionInput, -1, 1));
    }
}