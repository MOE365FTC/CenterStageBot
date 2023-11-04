package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

@Autonomous
public class blueLeftRR extends LinearOpMode {
    MOEBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(-62,-12,0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                //vision
                .lineToLinearHeading(new Pose2d(-33,12, Math.toRadians(90)))
                .waitSeconds(0.25)
                //spike deposit
                .lineToLinearHeading(new Pose2d(-45,14, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-46,30), Math.toRadians(90))
                .splineTo(new Vector2d(-32,46), Math.toRadians(90))
                .waitSeconds(0.25)
                //pixel backdrop dispense
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                //vision
                .lineToLinearHeading(new Pose2d(-33,12, Math.toRadians(0)))
                .waitSeconds(0.25)
                //spike deposit
                .lineToLinearHeading(new Pose2d(-45,14, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-46,30), Math.toRadians(90))
                .splineTo(new Vector2d(-32,46), Math.toRadians(90))
                .waitSeconds(0.25)
                //pixel backdrop dispense
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                //vision
                .lineToLinearHeading(new Pose2d(-33,12, Math.toRadians(-90)))
                .waitSeconds(0.25)
                //spike deposit
                .lineToLinearHeading(new Pose2d(-45,14, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-46,30), Math.toRadians(90))
                .splineTo(new Vector2d(-32,46), Math.toRadians(90))
                .waitSeconds(0.25)
                //pixel backdrop dispense
                .build();




        TrajectorySequence blueCycle = drive.trajectorySequenceBuilder(new Pose2d(-32, 46))
                .lineTo(new Vector2d(-36, -60))
                .lineTo(new Vector2d(-36, 17))
                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
                .waitSeconds(0.25)
                //pixel backdrop output
                .lineTo(new Vector2d(-36, -60))
                .lineTo(new Vector2d(-36, 17))
                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
                .waitSeconds(0.25)
                //pixel backdrop output
                .strafeLeft(28)
                .forward(10)
                .build();

        while(!isStarted() && !isStopRequested()) {
            //init loop

            robot.vision.detectProp();
            telemetry.addData("Prop Pos", robot.vision.getPropPos());
        }

        waitForStart();
        robot.vision.stopDetecting();


    }
}
