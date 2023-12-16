package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.backup.backupHardware.BackupMOEBot;
import org.firstinspires.ftc.teamcode.hardware.DispenserDec17;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class PowerPlayBlueRightCenter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

        Pose2d startPose = new Pose2d(-62, -35, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-33, -35, Math.toRadians(90))) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonLeftIris(false);
                })
                .lineToConstantHeading(new Vector2d(8, -35))
                .forward(20)
//                                .splineTo(new Vector2d(8, -3), Math.toRadians(90)) //go to backdrop: go to gate with motor pushing center
//                                .splineToConstantHeading(new Vector2d(-10,28), Math.toRadians(90)) //go to backdrop: avoid teammate's pixels
                .splineToLinearHeading(new Pose2d(-35,48, Math.toRadians(90)), Math.toRadians(180)) //go to backdrop
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .waitSeconds(0.1)
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                //score
                .strafeRight(12) //park: dont run into board
                .splineToConstantHeading(new Vector2d(-13, 63), Math.toRadians(90)) //park
                .build();

        TrajectorySequence pixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-33, -35)) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonLeftIris(false);
                })
                .strafeRight(18)
                .forward(20)
                .splineTo(new Vector2d(8, -3), Math.toRadians(90)) //go to backdrop: go to gate with motor pushing center
//                                .splineToConstantHeading(new Vector2d(-10,28), Math.toRadians(90)) //go to backdrop: avoid teammate's pixels
                .splineToLinearHeading(new Pose2d(-35,48,Math.toRadians(90)), Math.toRadians(180)) //go to backdrop
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .waitSeconds(0.1)
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .strafeRight(12) //park: dont run into board
                .splineToConstantHeading(new Vector2d(-13, 63), Math.toRadians(90)) //park
                .build();

        TrajectorySequence pixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30, -35, Math.toRadians(-90))) //goto tick marks
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> { //place pixel on tick mark
                    robot.dispenser.autonLeftIris(false);
                })
                .strafeLeft(15)
                .lineToLinearHeading(new Pose2d(8, -35, Math.toRadians(90)))
                .forward(20)
//                                .splineTo(new Vector2d(8, -3), Math.toRadians(90)) //go to backdrop: go to gate with motor pushing center
//                                .splineToConstantHeading(new Vector2d(-10,28), Math.toRadians(90)) //go to backdrop: avoid teammate's pixels
                .splineToLinearHeading(new Pose2d(-35,48, Math.toRadians(90)), Math.toRadians(180)) //go to backdrop
                .waitSeconds(0.25) //stop momentum
                .addTemporalMarker(() -> {
                    robot.dispenser.autonLift(DispenserDec17.autonLiftPositions.LOW);
                })
                .waitSeconds(0.1)
                .forward(5)
                .addTemporalMarker(() -> {
                    robot.dispenser.autonRightIris(false);
                })
                .waitSeconds(0.1)
                .back(5)
                .strafeRight(12) //park: dont run into board
                .splineToConstantHeading(new Vector2d(-13, 63), Math.toRadians(90)) //park
                .build();

        while(!isStarted() && !isStopRequested()) {
            robot.dispenser.autonIris(true);

            robot.vision.detectProp();
            telemetry.addData("Prop Pos", robot.vision.getPropPos());
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
