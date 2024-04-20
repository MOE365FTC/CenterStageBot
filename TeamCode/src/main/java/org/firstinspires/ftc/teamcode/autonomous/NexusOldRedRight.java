package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class NexusOldRedRight extends LinearOpMode {
    //UPDATED FOR WORLDS : COMPLETELY UNTESTED
    MOEBot robot;
    public static int tiltTarget;
    public static final int slowerStartingVelocity = 30;
    public static double bufferTime = 0.3;
    public static int armOffset = 150;
    boolean usingVedic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, true, false);

        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(false);
                })
                .lineToLinearHeading(new Pose2d(23, 23, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .back(10)
                .lineToLinearHeading(new Pose2d(11.5, 32, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
                })
                .waitSeconds(0.75)
                .lineToSplineHeading(new Pose2d(42.5,39,Math.toRadians(-90))) //spline to backdrop left position
                .waitSeconds(0.4) //inertia
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(58, 48))
                .lineToLinearHeading(new Pose2d(60,60, Math.toRadians(-90))) //park
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(false);
                })
                .lineToLinearHeading(new Pose2d(15, 12, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .lineToLinearHeading(new Pose2d(4, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(11.5, 32, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch);
                })
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(34,32,Math.toRadians(-90))) //spline to backdrop center position
                .waitSeconds(0.4) //inertia
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltStraight + armOffset - 50; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
                })
                .waitSeconds(0.75)
                .addTemporalMarker(() -> {
                    robot.arm.autonExtend(460);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.75)
                .addTemporalMarker(() -> {
                    robot.arm.autonExtend(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(58, 48))
                .lineToLinearHeading(new Pose2d(60,60, Math.toRadians(-90))) //park
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(false);
                })
                .lineTo(new Vector2d(33, 10)) //go to tick mark
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .back(6)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
                })
                .waitSeconds(0.75)
                .lineToLinearHeading(new Pose2d(26,38,Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.4) //inertia
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(58, 48))
                .lineToLinearHeading(new Pose2d(60,60, Math.toRadians(-90))) //park
                .build();


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
                drive.followTrajectorySequenceAsync(pixelLeft);
                break;
            case 2:
                drive.followTrajectorySequenceAsync(pixelCenter);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(pixelRight);
                break;
        }
//
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            robot.arm.tiltArm(tiltTarget);
        }

    }
}
