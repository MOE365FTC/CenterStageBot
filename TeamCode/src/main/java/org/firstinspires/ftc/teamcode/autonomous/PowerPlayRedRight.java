package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DispenserDec17;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class PowerPlayRedRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
                })
                .forward(32)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(50, 20))
                .lineToLinearHeading(new Pose2d(34, 48, Math.toRadians(90)))
                //lift
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.PRE_INTAKE);
                })
                .strafeRight(26) //park: dont run into board
                .forward(7) //park
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
                })
                .lineToLinearHeading(new Pose2d(28, 12, Math.toRadians(90)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.5)
                .strafeRight(15)
                .lineToConstantHeading(new Vector2d(55, 30))
                .lineToLinearHeading(new Pose2d(40, 48, Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.PRE_INTAKE);
                })
                .strafeRight(12) //park: dont run into board
                .forward(7) //park
                .build();

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
                })
                .lineToLinearHeading(new Pose2d(30, 8, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(50, 20))
                .lineToLinearHeading(new Pose2d(28, 48, Math.toRadians(90)))
                //lift
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.PRE_INTAKE);
                })
                .strafeRight(30) //park: dont run into board
                .forward(7) //park
                .build();


        while(!isStarted() && !isStopRequested()) {
            robot.dispenser.autonIris(false);

            robot.vision.detectProp();
            telemetry.addData("Prop Pos", robot.vision.getPropPos());
            telemetry.addData("Status", "READY");
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