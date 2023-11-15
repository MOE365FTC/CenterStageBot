package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class redRightRR extends LinearOpMode {
    MOEBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(62,12,Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                //vision
                .lineToLinearHeading(new Pose2d(33,12, Math.toRadians(0)))
                .waitSeconds(0.25)
                //spike deposit
                .forward(15)
                .lineToLinearHeading(new Pose2d(43,46, Math.toRadians(90)))
                .waitSeconds(0.25)
                //backdrop
                .lineTo(new Vector2d(55,46))
                .splineToConstantHeading(new Vector2d(60,60), Math.toRadians(90))
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                //vision
                .lineToLinearHeading(new Pose2d(35,12, Math.toRadians(-90)))
                .waitSeconds(0.25)
                //spike deposit
                .back(15)
                .lineToLinearHeading(new Pose2d(35,46, Math.toRadians(90)))
                .waitSeconds(0.25)
                //backdrop
                .lineTo(new Vector2d(55,46))
                .splineToConstantHeading(new Vector2d(60,60), Math.toRadians(90))
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                ///vision
                .lineToLinearHeading(new Pose2d(33,12, Math.toRadians(180)))
                //spike deposit
                .lineToLinearHeading(new Pose2d(29,46, Math.toRadians(90)))
                .waitSeconds(0.25)
                //pixel backdrop dispense
                .lineTo(new Vector2d(55,46))
                .splineToConstantHeading(new Vector2d(60,60), Math.toRadians(90))
                .build();


        while(!isStarted() && !isStopRequested()) {
            //init loop

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
