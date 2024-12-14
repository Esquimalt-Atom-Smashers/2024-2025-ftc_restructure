package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.libs.roadrunner.RoadrunnerConstants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.libs.roadrunner.Mecanum20025;

@Autonomous(name="DriveBasePickingNeutral", group = "Real")
public class DriveBasePickingNeutral extends LinearOpMode {
    Mecanum20025 roadrunnerDrive;

    class roadrunnerPoseTele implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {
            if (roadrunnerDrive.pose != null) {
                telemetry.addData("x", roadrunnerDrive.pose.position.x);
                telemetry.addData("y", roadrunnerDrive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(roadrunnerDrive.pose.heading.toDouble()));
                telemetry.update();
            } else {
                telemetry.addData("Error", "Pose is null");
                telemetry.update();
            }
            return true;
        }
    }

    public Action roadrunnerUpdatePose() {
        return new roadrunnerPoseTele();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        Pose2d initialPose = new Pose2d(-3 * RoadrunnerConstants.INCH_TO_TILE, 1.5 * RoadrunnerConstants.INCH_TO_TILE, Math.toRadians(90));
        Mecanum20025 roadrunnerDrive = new Mecanum20025(hardwareMap, initialPose);
        Pose2d updatedPose = new Pose2d(roadrunnerDrive.pose.position.x, roadrunnerDrive.pose.position.y, roadrunnerDrive.pose.heading.toDouble());

        TrajectoryActionBuilder getNeutralBlock1 = roadrunnerDrive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-0.5 * RoadrunnerConstants.INCH_TO_TILE, 1.5 * RoadrunnerConstants.INCH_TO_TILE))
                .splineToConstantHeading(new Vector2d(-0.5 * RoadrunnerConstants.INCH_TO_TILE, 2 * RoadrunnerConstants.INCH_TO_TILE), Math.toRadians(0))
                .strafeTo(new Vector2d(-2.9 * RoadrunnerConstants.INCH_TO_TILE, 2.5 * RoadrunnerConstants.INCH_TO_TILE));
        Action getBlock1 = getNeutralBlock1.build();

        TrajectoryActionBuilder getNeutralBlock2 = roadrunnerDrive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-0.5 * RoadrunnerConstants.INCH_TO_TILE, 1.5 * RoadrunnerConstants.INCH_TO_TILE))
                .splineToConstantHeading(new Vector2d(-0.5 * RoadrunnerConstants.INCH_TO_TILE, 2.5 * RoadrunnerConstants.INCH_TO_TILE), Math.toRadians(0))
                .strafeTo(new Vector2d(-2.9 * RoadrunnerConstants.INCH_TO_TILE, 2.5 * RoadrunnerConstants.INCH_TO_TILE));
        Action getBlock2 = getNeutralBlock2.build();

        TrajectoryActionBuilder getNeutralBlock3 = roadrunnerDrive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-0.5 * RoadrunnerConstants.INCH_TO_TILE, 1.5 * RoadrunnerConstants.INCH_TO_TILE))
                .splineToConstantHeading(new Vector2d(-0.5 * RoadrunnerConstants.INCH_TO_TILE, 2.9 * RoadrunnerConstants.INCH_TO_TILE), Math.toRadians(0))
                .strafeTo(new Vector2d(-2.9 * RoadrunnerConstants.INCH_TO_TILE, 2.5 * RoadrunnerConstants.INCH_TO_TILE));
        Action getBlock3 = getNeutralBlock3.build();

        TrajectoryActionBuilder parkObzone = roadrunnerDrive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-2.5 * RoadrunnerConstants.INCH_TO_TILE, -2.5 * RoadrunnerConstants.INCH_TO_TILE));//roadrunnerDrive straight into ob zone

        Action chosenParkingAction = parkObzone.build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
            roadrunnerDrive.updatePoseEstimate();
            updatedPose = new Pose2d(roadrunnerDrive.pose.position.x, roadrunnerDrive.pose.position.y, roadrunnerDrive.pose.heading.toDouble());

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    getBlock1,
                                    getBlock2,
                                    getBlock3,
                                    chosenParkingAction
                            ),
                            roadrunnerUpdatePose()
                    )
            );//execute the planned action
        }
    }
}
