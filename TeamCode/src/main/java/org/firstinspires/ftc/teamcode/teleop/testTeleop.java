package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.hardware.Outtake;

@TeleOp(group = "test")
public class testTeleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, false, false);
    }

    @Override
    public void init_loop(){
        robot.chassis.imuTelemetry(telemetry);
        if(gamepad2.right_stick_button) {
            robot.arm.resetEncoders();
        }
    }

    @Override
    public void loop() {
        robot.chassis.fieldCentricDrive();
        robot.chassis.imuTelemetry(telemetry);
        robot.chassis.odoTelemetry(telemetry);
        robot.arm.actuate();
        robot.arm.telemetryArm();
        robot.droneLauncher.actuate();

//        if(gamepad2.right_stick_button) {
//            robot.arm.resetEncoders();
//        }
    }
}
