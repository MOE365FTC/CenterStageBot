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

@Autonomous
public class NexusNewRedLeft extends LinearOpMode {
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
                .lineToConstantHeading(new Vector2d(34.5,-34), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .lineToLinearHeading(new Pose2d(11,-44.5, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(10, 39))
                .lineToConstantHeading(new Vector2d(10, 60))
//                .addTemporalMarker(() -> {
//                    robot.arm.autonRunIntake(true); //run intake inwards
//                })
//                .lineToLinearHeading(new Pose2d(11.5,-60, Math.toRadians(-90)))
//                .forward(8.5, SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity -15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    robot.arm.runGrabs(true); //run grabbers inward
//                })
//                .waitSeconds(1.0)
//                .lineToConstantHeading(new Vector2d(11.5, 30))
//                .addTemporalMarker(() -> {
//                    robot.arm.autonRunIntake(false); //stop intake
//                    robot.arm.autonSetBoxGate(false); //close intake box
//                    robot.arm.autonSetPitchServo(Arm.basePitch);
//                })
//                .waitSeconds(0.5)
//                .lineToLinearHeading(new Pose2d(31.5,30,Math.toRadians(-90))) //spline to backdrop center position
//                .waitSeconds(0.4) //inertia
//                .addTemporalMarker(() -> {
//                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
//                })
//                .waitSeconds(0.75)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(330);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(true); //open scoring box
//                })
//                .waitSeconds(0.17)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(false); //open scoring box
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(0);
//                })
//                .lineToLinearHeading(new Pose2d(42.5,30,Math.toRadians(-90))) //spline to backdrop center position
//                .waitSeconds(0.4)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(330);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(true); //open scoring box
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(0);
//                })
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
//                })
//                .waitSeconds(0.75)
//                .lineToConstantHeading(new Vector2d(8, 39))
//                .lineToConstantHeading(new Vector2d(8, 60))
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(6)
                .lineToConstantHeading(new Vector2d(15,-43.5), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .lineToConstantHeading(new Vector2d(11,-43.5))
                .lineToConstantHeading(new Vector2d(10, 39))
                .lineToConstantHeading(new Vector2d(10, 60))
//                .addTemporalMarker(() -> {
//                    robot.arm.autonRunIntake(true); //run intake inwards
//                })
//                .lineToLinearHeading(new Pose2d(11.5,-60, Math.toRadians(-90)))
//                .forward(8.5, SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity -15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    robot.arm.runGrabs(true); //run grabbers inward
//                })
//                .waitSeconds(1.0)
//                .lineToConstantHeading(new Vector2d(11.5, 30))
//                .addTemporalMarker(() -> {
//                    robot.arm.autonRunIntake(false); //stop intake
//                    robot.arm.autonSetBoxGate(false); //close intake box
//                    robot.arm.autonSetPitchServo(Arm.basePitch);
//                })
//                .waitSeconds(0.5)
//                .lineToLinearHeading(new Pose2d(31.5,30,Math.toRadians(-90))) //spline to backdrop center position //35.5
//                .waitSeconds(0.4) //inertia
//                .addTemporalMarker(() -> {
//                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
//                })
//                .waitSeconds(0.75)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(330);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(true); //open scoring box
//                })
//                .waitSeconds(0.17)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(false); //open scoring box
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(0);
//                })
//                .lineToLinearHeading(new Pose2d(37,30,Math.toRadians(-90))) //spline to backdrop center position
//                .waitSeconds(0.4)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(330);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(true); //open scoring box
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(0);
//                })
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
//                })
//                .waitSeconds(0.75)
//                .lineToConstantHeading(new Vector2d(8, 39))
//                .lineToConstantHeading(new Vector2d(8, 60))
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(6)
                .lineToConstantHeading(new Vector2d(24,-49), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .lineToConstantHeading(new Vector2d(11,-46))
                .lineToConstantHeading(new Vector2d(10, 39))
                .lineToConstantHeading(new Vector2d(10, 60))
//                .addTemporalMarker(() -> {
//                    robot.arm.autonRunIntake(true); //run intake inwards
//                })
//                .lineToLinearHeading(new Pose2d(11.5,-60, Math.toRadians(-90)))
//                .forward(8.5, SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity -15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(() -> {
//                    robot.arm.runGrabs(true); //run grabbers inward
//                })
//                .waitSeconds(1.0)
//                .lineToConstantHeading(new Vector2d(11.5, 30))
//                .addTemporalMarker(() -> {
//                    robot.arm.autonRunIntake(false); //stop intake
//                    robot.arm.autonSetBoxGate(false); //close intake box
//                    robot.arm.autonSetPitchServo(Arm.basePitch);
//                })
//                .waitSeconds(0.5)
//                .lineToLinearHeading(new Pose2d(42.5,30,Math.toRadians(-90))) //spline to backdrop center position
//                .waitSeconds(0.4) //inertia
//                .addTemporalMarker(() -> {
//                    tiltTarget = Arm.tiltStraight + armOffset; //tilt arm to scoring position
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetPitchServo(Arm.autonScorePitch); //set pitch servo to scoring position
//                })
//                .waitSeconds(0.75)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(330);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(true); //open scoring box
//                })
//                .waitSeconds(0.17)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(false); //open scoring box
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(0);
//                })
//                .lineToLinearHeading(new Pose2d(31.5,30,Math.toRadians(-90))) //spline to backdrop center position
//                .waitSeconds(0.4)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(330);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetBoxGate(true); //open scoring box
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.arm.autonExtend(0);
//                })
//                .addTemporalMarker(() -> {
//                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
//                })
//                .waitSeconds(0.75)
//                .lineToConstantHeading(new Vector2d(8, 39))
//                .lineToConstantHeading(new Vector2d(8, 60))
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
