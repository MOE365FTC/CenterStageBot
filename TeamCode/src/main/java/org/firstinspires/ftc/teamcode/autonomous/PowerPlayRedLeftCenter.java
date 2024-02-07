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
public class PowerPlayRedLeftCenter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        boolean prevLeftBumper = false, prevRightBumper = false;
        boolean delaySet = false;
        int delaySeconds = 0;
        int MAX_DELAY_TIME = 10;

        Pose2d startPose = new Pose2d(62, -39, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        while(!isStarted() && !isStopRequested() && !delaySet) {
            if(gamepad2.right_bumper && !prevRightBumper)
                delaySeconds ++;
            else if(gamepad2.left_bumper && !prevLeftBumper)
                delaySeconds --;
            prevLeftBumper = gamepad2.left_bumper;
            prevRightBumper = gamepad2.right_bumper;
            delaySeconds = Math.max(Math.min(delaySeconds, MAX_DELAY_TIME), 0);

            if(gamepad2.a)
                delaySet = true;

            telemetry.addData("Delay", delaySeconds);
            telemetry.addLine("Press A on Gamepad2 to set delay");
            telemetry.update();
        }
        telemetry.addLine("Building Path");
        telemetry.update();
        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(delaySeconds)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(31, -44, Math.toRadians(-90))) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-1, -38, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-1, 20))
//                                .splineTo(new Vector2d(8, -3), Math.toRadians(90)) //go to backdrop: go to gate with motor pushing center
//                                .splineToConstantHeading(new Vector2d(-10,28), Math.toRadians(90)) //go to backdrop: avoid teammate's pixels
                .splineToLinearHeading(new Pose2d(33,44, Math.toRadians(90)), Math.toRadians(180)) //go to backdrop
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> {
                                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .waitSeconds(0.1)
                .forward(5)
                .addTemporalMarker(() -> {
                                    robot.dispenser.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(() -> {
                                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.PRE_INTAKE);
                })
                .strafeLeft(24) //park: dont run into board
                .forward(7) //park
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(delaySeconds)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(31, -40)) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.25)
                .strafeLeft(18)
                .lineToConstantHeading(new Vector2d(-1, -58))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-1, 20))
                .splineToLinearHeading(new Pose2d(35,43,Math.toRadians(90)), Math.toRadians(180)) //go to backdrop
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .waitSeconds(0.1)
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.PRE_INTAKE);
                })
                .strafeLeft(26) //park: dont run into board
                .forward(7) //park
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(delaySeconds)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonIris(true);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.AUTON_INTAKE);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(29, -38, Math.toRadians(90))) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.25)
                .back(4)
                .lineToConstantHeading(new Vector2d(-1, -38))
                .lineToConstantHeading(new Vector2d(-1, 20))
//                                .splineTo(new Vector2d(8, -3), Math.toRadians(90)) //go to backdrop: go to gate with motor pushing center
//                                .splineToConstantHeading(new Vector2d(-10,28), Math.toRadians(90)) //go to backdrop: avoid teammate's pixels
                .splineToLinearHeading(new Pose2d(40,44, Math.toRadians(90)), Math.toRadians(180)) //go to backdrop
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .waitSeconds(0.1)
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLeftIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.PRE_INTAKE);
                })
                .strafeLeft(30) //park: dont run into board
                .forward(7) //park
                .build();

        while(!isStarted() && !isStopRequested()) {
            robot.dispenser.autonIris(false);

            robot.visionTensorflow.detectProp();
            telemetry.addData("Prop Pos", robot.visionTensorflow.getPropPos());
            telemetry.addData("Status", "READY");
        }

        waitForStart();
        robot.visionTensorflow.stopDetecting();

        switch(robot.visionTensorflow.getPropPos()){
            case 1:
                robot.dispenser.autonIris(true);
                drive.followTrajectorySequence(pixelLeft);
                break;
            case 2:
                robot.dispenser.autonIris(true);
                drive.followTrajectorySequence(pixelCenter);
                break;
            case 3:
                robot.dispenser.autonIris(true);
                drive.followTrajectorySequence(pixelRight);
                break;
        }


    }
}
