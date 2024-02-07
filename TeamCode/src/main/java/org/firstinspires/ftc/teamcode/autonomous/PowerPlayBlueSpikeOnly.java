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
public class PowerPlayBlueSpikeOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        boolean prevLeftBumper = false, prevRightBumper = false;
        boolean delaySet = false;
        int delaySeconds = 0;
        int MAX_DELAY_TIME = 10;

        Pose2d startPose = new Pose2d(-62, -39, Math.toRadians(0));
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
                .lineToLinearHeading(new Pose2d(-31, -35, Math.toRadians(90))) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .forward(2)
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.25)
                .back(4)
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
                .lineToConstantHeading(new Vector2d(-31, -35)) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.25)
                .back(4)
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
                .lineToLinearHeading(new Pose2d(-30, -37, Math.toRadians(-90))) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.25)
                .back(4)
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
