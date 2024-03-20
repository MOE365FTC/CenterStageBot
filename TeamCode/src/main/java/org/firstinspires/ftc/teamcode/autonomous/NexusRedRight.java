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

@Autonomous
public class NexusRedRight extends LinearOpMode {
    MOEBot robot;
    public static int tiltTarget;
    public static double bufferTime = 0.3;
    boolean usingVedic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, true, false);

        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(23, 24, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(0.5) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .lineToLinearHeading(new Pose2d(16, 38, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(37.5,38,Math.toRadians(-90))) //spline to backdrop left position
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.SCORE);
                })
                .waitSeconds(0.5)
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
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                })
                .waitSeconds(1.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.HOVER);
                })
                .waitSeconds(2.0) //buffer time for outtake
                .lineToConstantHeading(new Vector2d(66, 54))
                .lineToLinearHeading(new Pose2d(70,60, Math.toRadians(-90))) //park
                .addTemporalMarker(() -> {
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.EXTENDED_FULL);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.intake.setTransferBeltServo(true);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(1.5)
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15, 40, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(16, 7, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(0.5) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .back(4)
                .lineToSplineHeading(new Pose2d(37,38,Math.toRadians(-90))) //spline to backdrop left position
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.SCORE);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.45);
                    robot.outtake.setYawServo(0.66);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.EXTENDED_FULL);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                    robot.intake.setTransferBeltServo(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.outtake.setYawServo(0.32);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                })
                .waitSeconds(1.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(2.0) //buffer time for outtake
                .lineToConstantHeading(new Vector2d(66, 54))
                .lineToLinearHeading(new Pose2d(70,60, Math.toRadians(-90))) //park
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(33, 20, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(33, 6.5, Math.toRadians(-90))) //go to tick mark
                .addTemporalMarker(() -> {
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(0.5) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .back(4)
                .lineToSplineHeading(new Pose2d(24.5,38,Math.toRadians(-90))) //spline to backdrop left position
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.SCORE);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.45);
                    robot.outtake.setYawServo(0.66);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.EXTENDED_FULL);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(false);
                    robot.intake.setTransferBeltServo(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.outtake.setYawServo(0.32);
                    robot.intake.autonIntakeSlides(Intake.ExtendPositions.TRANSFER);
                })
                .waitSeconds(1.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(2.0) //buffer time for outtake
                .lineToConstantHeading(new Vector2d(66, 54))
                .lineToLinearHeading(new Pose2d(70,60, Math.toRadians(-90))) //park
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
                robot.outtake.autonIris(true);
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
