//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.hardware.DispenserDec17;
//import org.firstinspires.ftc.teamcode.hardware.MOEBot;
//import org.firstinspires.ftc.teamcode.hardware.Outtake;
//import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
//
//@Deprecated
//@Autonomous
//public class ProtoAutoVision extends LinearOpMode {
//
//    boolean usingVedic = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);
//
//        Pose2d startPose = new Pose2d(-62, 12, Math.toRadians(0));
//        drive.setPoseEstimate(startPose);
//
//        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonIris(true);
//                })
//                .waitSeconds(1.0)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
//                })
//                .forward(31)
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonRightIris(false);
//                })
//                .waitSeconds(0.5)
//                .lineToConstantHeading(new Vector2d(-50, 20))
//                .lineToLinearHeading(new Pose2d(-29, 48, Math.toRadians(90)))
//                //lift
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
//                })
//                .forward(5)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonLeftIris(false);
//                })
//                .waitSeconds(0.1)
//                .back(5)
//                .strafeLeft(26) //park: dont run into board
//                .forward(7) //park
//                .build();
//
//        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonIris(true);
//                })
//                .waitSeconds(1.0)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
//                })
//                .lineToLinearHeading(new Pose2d(-29, 10, Math.toRadians(-90)))
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonRightIris(false);
//                })
//                .waitSeconds(0.5)
//                .lineToConstantHeading(new Vector2d(-50, 20))
//                .lineToLinearHeading(new Pose2d(-25, 48, Math.toRadians(90)))
//                //lift
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
//                })
//                .forward(5)
//                .addTemporalMarker(() -> {
//                    robot.dispenser.autonLeftIris(false);
//                })
//                .waitSeconds(0.1)
//                .back(5)
//                .strafeLeft(31) //park: dont run into board
//                .forward(7) //park
//                .build();
//
//        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(() -> {
//                    robot.outtake.autonIris(true);
//                })
//                .waitSeconds(1.0)
//                .addTemporalMarker(() -> {
//                    robot.outtake.autonLift(Outtake.autonLiftPositions.AUTON_INTAKE);
//                })
//                .lineToLinearHeading(new Pose2d(-29, 14, Math.toRadians(90)))
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.outtake.autonRightIris(false);
//                })
//                .waitSeconds(0.5)
//                .strafeLeft(15)
//                .lineToConstantHeading(new Vector2d(-55, 30))
//                .lineToLinearHeading(new Pose2d(-33, 48, Math.toRadians(90)))
//                .addTemporalMarker(() -> {
//                    robot.outtake.autonLift(Outtake.autonLiftPositions.LOW);
//                })
//                .forward(5)
//                .addTemporalMarker(() -> {
//                    robot.outtake.autonLeftIris(false);
//                })
//                .waitSeconds(0.1)
//                .back(5)
//                .strafeLeft(22) //park: dont run into board
//                .forward(7) //park
//                .build();
//
//
//        while(!isStarted() && !isStopRequested()) {
//            robot.outtake.autonIris(false);
//            if(!usingVedic) robot.visionTensorflow.detectProp();
//            if(gamepad2.a && !usingVedic) {
//                usingVedic = true;
//                robot.visionTensorflow.stopDetecting();
//                sleep(1500);
//                robot.visionBlob.initBlob();
//            } else if (gamepad2.a) {
//                usingVedic = false;
//                robot.visionBlob.stopDetecting();
//                robot.visionTensorflow.initTfod();
//            }
//            telemetry.addData("Vision System: ", usingVedic ? "Blob" : "AI");
//            telemetry.addData("Prop Pos", usingVedic? robot.visionBlob.getPropPos() : robot.visionTensorflow.getPropPos());
//            telemetry.addData("Status", "READY");
//            telemetry.update();
//        }
//
//        waitForStart();
//        if(usingVedic) robot.visionBlob.stopDetecting();
//        else robot.visionTensorflow.stopDetecting();
//
//        switch(robot.visionTensorflow.getPropPos()){
//            case 1:
//                drive.followTrajectorySequence(pixelLeft);
//                break;
//            case 2:
//                drive.followTrajectorySequence(pixelCenter);
//                break;
//            case 3:
//                drive.followTrajectorySequence(pixelRight);
//                break;
//        }
//
//    }
//}
