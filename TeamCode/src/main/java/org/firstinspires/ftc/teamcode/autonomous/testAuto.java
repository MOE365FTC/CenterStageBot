package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Autonomous
public class testAuto extends LinearOpMode {
    MOEBot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);
        while(!isStarted() && !isStopRequested()) {
            //init loop

            robot.vision.detectProp();
            telemetry.addData("Prop Pos", robot.vision.getPropPos());
        }
        waitForStart();
        robot.vision.stopDetecting();
    }
}
