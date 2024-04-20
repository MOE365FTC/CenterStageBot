package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Match Autons")
public class NexusRedLeft extends LinearOpMode {
    //UPDATED FOR WORLDS : PIXEL LEFT UNTESTED
    MOEBot robot;
    public static int tiltTarget = 0;
    public static final int slowerStartingVelocity = 30;
    public static int armOffset = 155;
    boolean usingVedic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, true, true);

        Pose2d startPose = new Pose2d(62, -38.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(6)
                .lineToConstantHeading(new Vector2d(34.5,-32), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .waitSeconds(1.0)
                .back(10)
                .lineToLinearHeading(new Pose2d(11,-35, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(true); //run intake inwards
                })
                .lineToLinearHeading(new Pose2d(11.5,-57.5, Math.toRadians(-90)))
                .forward(5.5, SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity -15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(true); //run grabbers inward
                })
                .waitSeconds(1.0)
                .lineToConstantHeading(new Vector2d(11.5, 32))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop intake
                    robot.arm.autonSetBoxGate(false); //close intake box
                    robot.arm.autonSetPitchServo(Arm.basePitch);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(21, 39))  //original y before making arm steeper: 42
                .waitSeconds(0.4) //inertia
                //might have to move into backdrop
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.17)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(false); //close scoring box
                })
                .waitSeconds(0.17)
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(35, 40.5))  //greater y cause compensating for heading drift w/ arm out
                .waitSeconds(0.2) //inertia
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(1.5, 39))
                .lineToConstantHeading(new Vector2d(1.5, 60))
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(6)
                .lineToConstantHeading(new Vector2d(15,-35), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .lineToConstantHeading(new Vector2d(11,-35))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(true); //run intake inwards
                })
                .lineToLinearHeading(new Pose2d(11.5,-57.5, Math.toRadians(-90)))
                .forward(4.5, SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity -15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(true); //run grabbers inward
                })
                .waitSeconds(1.0)
                .lineToConstantHeading(new Vector2d(11.5, 32))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop intake
                    robot.arm.autonSetBoxGate(false); //close intake box
                    robot.arm.autonSetPitchServo(Arm.basePitch);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(23, 39.5))  //original y before making arm steeper: 42
                .waitSeconds(0.4) //inertia
                //might have to move into backdrop
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.17)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(false); //open scoring box
                })
                .waitSeconds(0.17)
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(31, 39.5))  //original y before making arm steeper: 42
                .waitSeconds(0.2) //inertia
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(1.5, 42))
                .lineToConstantHeading(new Vector2d(1.5, 60))
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(6)
                .lineToConstantHeading(new Vector2d(24,-46), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .lineToConstantHeading(new Vector2d(11,-46))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(true); //run intake inwards
                })
                .lineToLinearHeading(new Pose2d(11.5,-57.5, Math.toRadians(-90)))
                .forward(5.5, SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity -15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(true); //run grabbers inward
                })
                .waitSeconds(1.0)
                .lineToConstantHeading(new Vector2d(11.5, 32))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop intake
                    robot.arm.autonSetBoxGate(false); //close intake box
                    robot.arm.autonSetPitchServo(Arm.basePitch);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(36, 41))
                .waitSeconds(0.4) //inertia
                //might have to move into backdrop
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.17)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(false); //close scoring box
                })
                .waitSeconds(0.17)
                .lineToConstantHeading(new Vector2d(26, 41))  //original y before making arm steeper: 42
                .waitSeconds(0.2) //inertia
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .waitSeconds(0.75)
                .lineToConstantHeading(new Vector2d(1.5, 42))
                .lineToConstantHeading(new Vector2d(1.5, 60))
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
