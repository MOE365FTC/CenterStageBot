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
public class NexusBlueRight extends LinearOpMode {
    MOEBot robot;
    public static int tiltTarget;
    public static double bufferTime = 0.3;
    boolean usingVedic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, true);

        Pose2d startPose = new Pose2d(-62, -39.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> { //preload pixels at the start
//                                            robot.outtake.autonIris(true); //expand irises to lock in pixels
//                                            robot.outtake.autonIntakeSlides(BASE) //set intake slides to base; NEEDS CHECKING
//                                            robot.outtake.autonLift(BASE); //set auton lift to base; OR liftTiltTarget = BASE; since pidf loop is used on this
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for outtake
                .lineToSplineHeading(new Pose2d(-31, -35, Math.toRadians(90))) //go to tick mark
                .addTemporalMarker(() -> { //place pixel on tick mark
//                                            robot.outtake.autonLift(LOW) //set auton lift to low so pixel wont bounce
//                                            robot.outtake.autonRightIris(false); //shrinks right iris such that the purple pixel will be dropped on tick mark
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for tick mark
                .lineToSplineHeading(new Pose2d(-24, -35, Math.toRadians(0))) //maintain fluidity and smoothness of path to intake white pixels
                .splineToSplineHeading(new Pose2d(-11.5,-45,Math.toRadians(-90)), Math.toRadians(-90)) //move to intake white pixels
                .addTemporalMarker(() -> { //intake white pixels
//                                            robot.outtake.autonIntakeSlides(EXTENDED_FULL) //set intake slides to full extension; NEEDS CHECKING
//                                            robot.outtake.intakeMotor(run) //set intake motor to run to intake white pixels; NEEDS CREATION
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for intake
                .lineTo(new Vector2d(-12, 25)) //maintain directness of path to score on backdrop
                .splineToSplineHeading(new Pose2d(-42,48, Math.toRadians(-90)), Math.toRadians(90)) //spline to backdrop left position
                .addTemporalMarker(() -> { //score on backdrop
//                                            robot.outtake.tiltPID(autonLiftPositions.AUTON_SCORE); //set auton lift to score; NEEDS CHECKING
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for outtake
                .lineToSplineHeading(new Pose2d(-18, 38.5, Math.toRadians(-30))) //make sure not to collide with backdrop
                .splineToSplineHeading(new Pose2d(-12, 25,Math.toRadians(-90)), Math.toRadians(-90)) //spline into position to prepare to intake
                //hypothetically can start extending intake slides here
                .lineToLinearHeading(new Pose2d(-11.5,-45,Math.toRadians(-90))) //move to intake white pixels
                .addTemporalMarker(() -> { //intake white pixels
//                                            robot.outtake.autonIntakeSlides(EXTENDED_FULL) //set intake slides to full extension; NEEDS CHECKING
//                                            robot.outtake.intakeMotor(run) //set intake motor to run to intake white pixels; NEEDS CREATION
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for intake
                .lineTo(new Vector2d(-12, 25)) //maintain directness of path to score on backdrop
                .splineToSplineHeading(new Pose2d(-42,48,Math.toRadians(-90)), Math.toRadians(90)) //spline to backdrop left position
                .lineTo(new Vector2d(-15,48)) //avoid collision with backdrop
                .splineToSplineHeading(new Pose2d(-10,60, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> { //preload pixels at the start
//                                            robot.outtake.autonIris(true); //expand irises to lock in pixels
//                                            robot.outtake.autonIntakeSlides(BASE) //set intake slides to base; NEEDS CHECKING
//                                            robot.outtake.autonLift(BASE); //set auton lift to base; OR liftTiltTarget = BASE; since pidf loop is used on this
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for outtake
                .lineToSplineHeading(new Pose2d(-15, -35, Math.toRadians(0))) //maintain fluidity and smoothness of path to intake white pixels
                .addTemporalMarker(() -> { //place pixel on tick mark
//                                            robot.outtake.autonLift(LOW) //set auton lift to low so pixel wont bounce
//                                            robot.outtake.autonRightIris(false); //shrinks right iris such that the purple pixel will be dropped on tick mark
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for tick mark
                .splineToSplineHeading(new Pose2d(-11.5,-45,Math.toRadians(-90)), Math.toRadians(-90)) //move to intake white pixels
                .addTemporalMarker(() -> { //intake white pixels
//                                            robot.outtake.autonIntakeSlides(EXTENDED_FULL) //set intake slides to full extension; NEEDS CHECKING
//                                            robot.outtake.intakeMotor(run) //set intake motor to run to intake white pixels; NEEDS CREATION
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for intake
                .lineTo(new Vector2d(-12, 17)) //maintain directness of path to score on backdrop
                .splineToSplineHeading(new Pose2d(-35.5,48,Math.toRadians(-90)), Math.toRadians(90)) //spline to backdrop left position
                .addTemporalMarker(() -> { //score on backdrop
//                                            robot.outtake.tiltPID(autonLiftPositions.AUTON_SCORE); //set auton lift to score; NEEDS CHECKING
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for outtake
                .lineToSplineHeading(new Pose2d(-18, 38.5, Math.toRadians(-30))) //make sure not to collide with backdrop
                .splineToSplineHeading(new Pose2d(-12, 17,Math.toRadians(-90)), Math.toRadians(-90)) //spline into position to prepare to intake
                //hypothetically can start extending intake slides here
                .lineToLinearHeading(new Pose2d(-11.5,-45,Math.toRadians(-90))) //move to intake white pixels
                .addTemporalMarker(() -> { //intake white pixels
//                                            robot.outtake.autonIntakeSlides(EXTENDED_FULL) //set intake slides to full extension; NEEDS CHECKING
//                                            robot.outtake.intakeMotor(run) //set intake motor to run to intake white pixels; NEEDS CREATION
//                                            MORE WORK + TESTING NEEDED HERE
                })
                .waitSeconds(bufferTime) //buffer time for intake
                .lineTo(new Vector2d(-12, 17)) //maintain directness of path to score on backdrop
                .splineToSplineHeading(new Pose2d(-35.5,48,Math.toRadians(-90)), Math.toRadians(90)) //spline to backdrop left position
                .lineTo(new Vector2d(-15,48)) //avoid collision with backdrop
                .splineToSplineHeading(new Pose2d(-10,60, Math.toRadians(-90)), Math.toRadians(90)) //park
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-55,-48))
                .lineToConstantHeading(new Vector2d(-22,-48))
                .addTemporalMarker(() -> {
                    robot.intake.runGrabs(false);
                    robot.intake.autonRunIntake(true, true);
                })
                .waitSeconds(1.0) // TODO: TUNE THIS!
                .addTemporalMarker(() -> {
                    robot.intake.autonRunIntake(false);
                })
                .lineTo(new Vector2d(-12, -35.5))
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.HOVER);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
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
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    robot.outtake.autonIris(true);
                })
                .lineTo(new Vector2d(-12, 34)) //maintain directness of path to score on backdrop
                .lineToLinearHeading(new Pose2d(-26.5,35, Math.toRadians(-90))) //spline to backdrop right position
                .turn(Math.toRadians(-5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.STRAIGHT);
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
                .forward(4)
                .addTemporalMarker(() -> {
                    robot.outtake.setPitchServo(0.0);
                    robot.outtake.setYawServo(0.32);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.outtake.autonLift(Outtake.autonLiftPositions.TRANSFER);
                    robot.outtake.autonTilt(Outtake.autonTiltPositions.BASE);
                })
                .waitSeconds(5.0) //buffer time for outtake
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
            robot.outtake.autonIris(false);
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

        drive.followTrajectorySequence(pixelRight);

        switch(usingVedic ? robot.visionBlob.getPropPos() : robot.visionTensorflow.getPropPos()){
            case 1:
//                drive.followTrajectorySequenceAsync(pixelLeft);
                break;
            case 2:
//                drive.followTrajectorySequenceAsync(pixelCenter);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(pixelRight);
                break;
        }
//
//        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
//            drive.update();
//        }

    }
}
