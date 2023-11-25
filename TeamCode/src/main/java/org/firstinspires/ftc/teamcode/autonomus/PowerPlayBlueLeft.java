package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.backup.backupHardware.BackupMOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class PowerPlayBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        BackupMOEBot robot = new BackupMOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(-62, 12, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .forward(29)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.backupClaw.release();
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-50, 20))
                .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(90)))
                .forward(10)
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-33, 12))
                .turn(Math.toRadians(-90))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.backupClaw.release();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(90)))
                .forward(10)
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-33, 12))
                .turn(Math.toRadians(90))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.backupClaw.release();
                })
                .waitSeconds(0.5)
                .strafeRight(24)
                .lineToLinearHeading(new Pose2d(-10, 60, Math.toRadians(90)))
                .build();


        while(!isStarted() && !isStopRequested()) {
            robot.backupClaw.grab();

            robot.vision.detectProp();
            telemetry.addData("Prop Pos", robot.vision.getPropPos());
        }

        waitForStart();
        robot.vision.stopDetecting();

        switch(robot.vision.getPropPos()){
            case 1:
                drive.followTrajectorySequence(pixelLeft);
                break;
            case 2:
                drive.followTrajectorySequence(pixelCenter);
                break;
            case 3:
                drive.followTrajectorySequence(pixelRight);
                break;
        }

    }
}
