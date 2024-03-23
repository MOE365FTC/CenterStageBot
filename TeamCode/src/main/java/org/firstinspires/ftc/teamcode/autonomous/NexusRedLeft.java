package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Match Autons")
public class NexusRedLeft extends LinearOpMode {
    MOEBot robot;
    public static int tiltTarget;
    public static double bufferTime = 0.3;
    boolean usingVedic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, true, true);

        Pose2d startPose = new Pose2d(62, -39.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36,-39.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36,-32, Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(0.5) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .back(4)
                .lineTo(new Vector2d(9.5, -35))
                .turn(Math.toRadians(-182))
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.HOVER);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(true); //start running in intake direction
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.EXTENDED_FULL);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setYawServo(0.32);
                    robot.intake.autonRunIntake(true);
                })
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.intake.runGrabs(true);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    robot.intake.setTransferBeltServo(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                    robot.intake.autonRunIntake(false);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(true);
                })
                .lineTo(new Vector2d(12, 34)) //maintain directness of path to score on backdrop
                .lineToLinearHeading(new Pose2d(42,39, Math.toRadians(-90))) //spline to backdrop right position
                .turn(Math.toRadians(5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.SCORE);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.45);
                    robot.outtake.setYawServo(0.66);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.outtake.setYawServo(0.32);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(2.0) //buffer time for outtake
                .forward(6)
                .lineToLinearHeading(new Pose2d(4, 56, Math.toRadians(-90)))
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(18,-60))
                .lineToConstantHeading(new Vector2d(18,-38))
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(0.5) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .back(2)
                .lineTo(new Vector2d(12, -36.5))
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.HOVER);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(true); //start running in intake direction
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.EXTENDED_FULL);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setYawServo(0.32);
                    robot.intake.autonRunIntake(true);
                })
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.intake.runGrabs(true);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    robot.intake.setTransferBeltServo(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                    robot.intake.autonRunIntake(false);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(true);
                })
                .lineTo(new Vector2d(12, 34)) //maintain directness of path to score on backdrop
                .lineToLinearHeading(new Pose2d(26.5,34, Math.toRadians(-90))) //spline to backdrop right position
                .turn(Math.toRadians(5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.SCORE);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.45);
                    robot.outtake.setYawServo(0.66);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.outtake.setYawServo(0.32);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(2.0) //buffer time for outtake
                .forward(6)
                .lineToLinearHeading(new Pose2d(4, 56, Math.toRadians(-90)))
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22,-39.5))
                .lineToConstantHeading(new Vector2d(22,-48))
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(0.5) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .back(2)
                .lineTo(new Vector2d(12, -35.5))
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.HOVER);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(true); //start running in intake direction
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.EXTENDED_FULL);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setYawServo(0.32);
                    robot.intake.autonRunIntake(true);
                })
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.intake.runGrabs(true);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    robot.intake.setTransferBeltServo(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                    robot.intake.autonRunIntake(false);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(true);
                })
                .lineTo(new Vector2d(12, 34)) //maintain directness of path to score on backdrop
                .lineToLinearHeading(new Pose2d(23.5,34, Math.toRadians(-90))) //spline to backdrop right position
                .turn(Math.toRadians(5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.SCORE);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.45);
                    robot.outtake.setYawServo(0.66);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.outtake.setYawServo(0.32);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(2.0) //buffer time for outtake
                .forward(6)
                .lineToLinearHeading(new Pose2d(4, 56, Math.toRadians(-90)))
                .build();

//        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
//
//                .lineToLinearHeading(new Pose2d(-12,-40,Math.toRadians(-90)))
//                .lineTo(new Vector2d(-12, 17)) //maintain directness of path to score on backdrop
//                .splineToSplineHeading(new Pose2d(-35.5,36,Math.toRadians(-90)), Math.toRadians(90)) //spline to backdrop left position
//                .addTemporalMarker(() -> { //score on backdrop
////                                            robot.outtake.tiltPID(autonLiftPositions.AUTON_SCORE); //set auton lift to score; NEEDS CHECKING
////                                            MORE WORK + TESTING NEEDED HERE
//                })
//                .waitSeconds(bufferTime) //buffer time for outtake
//                .lineToSplineHeading(new Pose2d(-18, 38.5, Math.toRadians(-30))) //make sure not to collide with backdrop
////                                .splineToSplineHeading(new Pose2d(-12, 17,Math.toRadians(-90)), Math.toRadians(-90)) //spline into position to prepare to intake
//                //hypothetically can start extending intake slides here
////                                .lineToLinearHeading(new Pose2d(-11.5,-45,Math.toRadians(-90))) //move to intake white pixels
////                                .addTemporalMarker(() -> { //intake white pixels
//////                                            robot.outtake.autonIntakeSlides(EXTENDED_FULL) //set intake slides to full extension; NEEDS CHECKING
//////                                            robot.outtake.intakeMotor(run) //set intake motor to run to intake white pixels; NEEDS CREATION
//////                                            MORE WORK + TESTING NEEDED HERE
////                                })
////                                .waitSeconds(bufferTime) //buffer time for intake
////                                .lineTo(new Vector2d(-12, 17)) //maintain directness of path to score on backdrop
////                                .splineToSplineHeading(new Pose2d(-35.5,36,Math.toRadians(-90)), Math.toRadians(90)) //spline to backdrop left position
////                                .lineTo(new Vector2d(-15,48)) //avoid collision with backdrop
//                .splineToSplineHeading(new Pose2d(-10,60, Math.toRadians(-90)), Math.toRadians(90)) //park
//                .build();


        while(!isStarted() && !isStopRequested()) {
            if(!usingVedic) robot.visionTensorflow.detectProp();
            if(gamepad2.a && !usingVedic) {
                usingVedic = true;
                robot.visionTensorflow.stopDetecting();
                sleep(1500);
                robot.visionBlob.initBlob();
            } else if (gamepad2.a) {
                usingVedic = false;
                robot.visionBlob.stopDetecting();
                robot.visionTensorflow.initTfod();
            }
            telemetry.addData("Vision System: ", usingVedic ? "Blob" : "AI");
            telemetry.addData("Prop Pos", usingVedic? robot.visionBlob.getPropPos() : robot.visionTensorflow.getPropPos());
            telemetry.addData("Status", "READY");
            telemetry.update();
        }
//
        waitForStart();
        if(usingVedic) robot.visionBlob.stopDetecting();
        else robot.visionTensorflow.stopDetecting();

        switch(usingVedic ? robot.visionBlob.getPropPos() : robot.visionTensorflow.getPropPos()){
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
//
//        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
//            drive.update();
//        }

    }
}
