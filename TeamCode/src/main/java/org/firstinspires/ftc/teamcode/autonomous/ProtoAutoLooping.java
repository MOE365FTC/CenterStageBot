package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DispenserDec17;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class ProtoAutoLooping extends LinearOpMode {
    MOEBot robot;
    public static int tiltTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(-62, 12, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    tiltTarget = 150;
                })
                .forward(31)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.autonRightIris(false);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-50, 20))
                .lineToLinearHeading(new Pose2d(-29, 48, Math.toRadians(90)))
                //lift
                .addTemporalMarker(() -> {
                    tiltTarget = 510;
                })
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .strafeLeft(26) //park: dont run into board
                .forward(7) //park
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(pixelCenter);
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            robot.outtake.tiltPID(tiltTarget);
        }
    }
}
