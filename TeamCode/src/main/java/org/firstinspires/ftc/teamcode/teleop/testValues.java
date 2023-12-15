package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class testValues extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);
        robot.chassis.resetIMU();
    }

    @Override
    public void loop() {
        robot.chassis.imuTelemetry(telemetry);
        robot.chassis.fieldCentricDrive();
        robot.dispenser.telemetryLiftPosition();
    }
}
