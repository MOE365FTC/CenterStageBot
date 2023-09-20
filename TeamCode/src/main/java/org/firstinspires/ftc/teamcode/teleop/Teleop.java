package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

public class Teleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void init_loop(){
        robot.chassis.imuTelemetry(telemetry);
    }

    @Override
    public void loop() {
        robot.chassis.fieldCentricDrive();
    }
}
