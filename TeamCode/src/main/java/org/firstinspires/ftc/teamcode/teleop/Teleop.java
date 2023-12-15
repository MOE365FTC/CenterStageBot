package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.hardware.PoseStorage;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
//    SampleMecanumDrive drive;
    TrajectorySequence zigMacroBottom;
    TrajectorySequence zigMacroUp;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);

//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setPoseEstimate(PoseStorage.currentPose); //poseStorage does not update currently
    }

    @Override
    public void init_loop(){
        robot.chassis.imuTelemetry(telemetry);
    }

    @Override
    public void loop() {
        robot.chassis.imuTelemetry(telemetry);
        robot.chassis.odoTelemetry(telemetry);
        robot.chassis.fieldCentricDrive();
        robot.dispenser.actuate();
//        robot.droneLauncher.actuate();
        robot.hang.actuate();

//        if(gamepad1.dpad_down && !drive.isBusy() && drive.getPoseEstimate().getY() < 0) { //zig zag macro
//            zigMacroBottom = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(-60, -35, Math.toRadians(90)))
//                    .lineToConstantHeading(new Vector2d(-60, -13))
//                    .strafeRight(24)
//                    .forward(28)
//                    .build();
//            drive.followTrajectorySequenceAsync(zigMacroBottom);
//        } else if (gamepad1.dpad_down && !drive.isBusy() && drive.getPoseEstimate().getY() > 0) {
//            zigMacroUp = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                    .lineToLinearHeading(new Pose2d(-35, 5, Math.toRadians(270)))
//                    .lineToConstantHeading(new Vector2d(-35, -13))
//                    .strafeRight(24)
//                    .forward(28)
//                    .build();
//            drive.followTrajectorySequenceAsync(zigMacroUp);
//        } else if(gamepad1.dpad_down && drive.isBusy()) {
//            drive.breakFollowing();
//        }
    }
}
