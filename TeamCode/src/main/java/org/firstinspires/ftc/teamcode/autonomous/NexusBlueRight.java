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
public class NexusBlueRight extends LinearOpMode {
    MOEBot robot;
    public static int tiltTarget = 0;
    public static int slowerStartingVelocity = 15;
    boolean usingVedic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, true, true);

        Pose2d startPose = new Pose2d(-62, -39.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-32,-39.5, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-32, -33))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(-32,-35))
                .lineToLinearHeading(new Pose2d(-11.5, -54, Math.toRadians(-90)))
                //spike deposit incomplete; mechanical
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-11.5, 36))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(-41, 42))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(-11.5, 42))
                .lineToConstantHeading(new Vector2d(-11.5, 60))
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-15,-35), SampleMecanumDrive.getVelocityConstraint(slowerStartingVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(false);
                    robot.arm.autonRunIntake(true, true); //run intake backwards
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop running intake
                })
                .lineToConstantHeading(new Vector2d(-11,-35))
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(true); //run intake inwards
                })
                .lineTo(new Vector2d(-11.5,-54))
                .addTemporalMarker(() -> {
                    robot.arm.runGrabs(true); //run grabbers inward
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.arm.autonRunIntake(false); //stop intake
                    robot.arm.autonSetBoxGate(false); //close intake box
                })
                .lineToConstantHeading(new Vector2d(-11.5, 36))
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltScore; //tilt arm to scoring position
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.scorePitch); //set pitch servo to scoring position
                })
                .lineToConstantHeading(new Vector2d(-36, 42))
                //might have to move into backdrop
                .addTemporalMarker(() -> {
                    robot.arm.autonSetBoxGate(true); //open scoring box
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.arm.autonSetPitchServo(Arm.basePitch); //set pitch servo to intake position
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    tiltTarget = Arm.tiltBase; //tilt arm to intake position
                })
                .lineToConstantHeading(new Vector2d(-11.5, 42))
                .lineToConstantHeading(new Vector2d(-11.5, 60))
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-15,-46))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(-11,-46))
                .turn(Math.toRadians(90))
                .waitSeconds(0.25)
                //spike deposit incomplete; mechanical
                .lineTo(new Vector2d(-11.5,-54))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-11.5, 36))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(-27, 42))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(-11.5, 42))
                .lineToConstantHeading(new Vector2d(-11.5, 60))
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
