package org.firstinspires.ftc.teamcode.autonomus;

import android.graphics.LinearGradient;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueRightAuton extends LinearOpMode {

    MOEBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(-62, -35, 0);
        drive.setPoseEstimate(startPose);


        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-37,-35),Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .splineToSplineHeading(new Pose2d(-37, -55, Math.toRadians(90)), Math.toRadians(0))
                .lineTo(new Vector2d(-11.5,-54))
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-37,-35),Math.toRadians(0))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .splineToSplineHeading(new Pose2d(-11.5,-54, Math.toRadians(90)), Math.toRadians(0))
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(-180)), Math.toRadians(0))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .back(24)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(-11.5, -54))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-11.5, -54, Math.toRadians(90)))
                //spike deposit incomplete; mechanical

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
