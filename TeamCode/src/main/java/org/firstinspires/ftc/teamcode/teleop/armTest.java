package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class armTest extends OpMode {
    MOEBot robot;
    DcMotorEx tiltA;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, false, false);
    }

    @Override
    public void loop() {
        robot.arm.actuate();
        robot.arm.telemetryArm();
    }
}
