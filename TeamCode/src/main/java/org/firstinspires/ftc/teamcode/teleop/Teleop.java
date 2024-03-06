package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, false);
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
        robot.outtake.actuate();
        robot.intake.updateGrabs();
        robot.droneLauncher.actuate();
        robot.hang.actuate();
    }
}
