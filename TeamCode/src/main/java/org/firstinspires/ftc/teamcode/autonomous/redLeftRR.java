package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class redLeftRR extends LinearOpMode {
    MOEBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(62,-35, Math.toRadians(-180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(37,-35, Math.toRadians(90)))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .back(15)
                .lineTo(new Vector2d(11.5,-54))
                //intake one pixel
                .lineTo(new Vector2d(11.5, 30))
                .splineTo(new Vector2d(35, 46), Math.toRadians(90))
                //score
                .strafeLeft(10)
                .splineToConstantHeading(new Vector2d(13, 60), Math.toRadians(90))
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(34,-35))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .forward(20)
                .lineToLinearHeading(new Pose2d(11.5,-54, Math.toRadians(90)))
                //intake one pixel
                .lineTo(new Vector2d(11.5, 30))
                .splineTo(new Vector2d(30, 46), Math.toRadians(90))
                //score
                .strafeLeft(5)
                .splineToConstantHeading(new Vector2d(13, 60), Math.toRadians(90))
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .lineToLinearHeading(new Pose2d(11.5, -54, Math.toRadians(90)))
                //intake one pixel
                .lineTo(new Vector2d(11.5, 30))
                .splineTo(new Vector2d(43, 46), Math.toRadians(90))
                //score
                .strafeLeft(20)
                .splineToConstantHeading(new Vector2d(13, 60), Math.toRadians(90))
                .build();


        while(!isStarted() && !isStopRequested()) {
            //init loop

            robot.visionTensorflow.detectProp();
            telemetry.addData("Prop Pos", robot.visionTensorflow.getPropPos());
        }

        waitForStart();
        robot.visionTensorflow.stopDetecting();

        switch(robot.visionTensorflow.getPropPos()) {
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
