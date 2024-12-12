package isaac_examples.main.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystemExample extends SubsystemBase {
    /** Define constants only used by the intake subsystem here. */
     private static class IntakeConstants {
        public static final String INTAKE_SERVO_NAME = "intakeServo";
        public static final String WRIST_SERVO_NAME = "wristServo";

        public static final double INTAKE_SERVO_POSITION = 0.0;
        public static final double OUTTAKE_SERVO_POSITION = 0.5;

        public static final double INTAKE_POWER = 0.5;
        public static final double OUTTAKE_POWER = 0.5;
    }

    /** Define the subsystems hardware here. */
    private CRServo intakeServo;
    private Servo wristServo;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    /** Initialize all hardware and any other fields in the constructor. */
    public IntakeSubsystemExample(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intakeServo = hardwareMap.get(CRServo.class, IntakeConstants.INTAKE_SERVO_NAME);
        wristServo = hardwareMap.get(Servo.class, IntakeConstants.WRIST_SERVO_NAME);
    }




}
