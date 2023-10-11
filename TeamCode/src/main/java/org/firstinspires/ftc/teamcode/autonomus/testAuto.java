package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class testAuto extends LinearOpMode {
    MOEBot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);
//        while(!isStarted() && !isStopRequested()) {
//            //init loop
//            telemetry.addData("1: ", "reached");
//            telemetry.update();
////            robot.vison.detectProp();
//        }
        waitForStart();
//        robot.vison.stopDetecting();
    }
}
