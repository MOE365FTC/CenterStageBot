package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "test")
public class testSpeed extends OpMode {

    DcMotor motor;
    double power = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
        if (gamepad1.y)
            power = 0.1;
        else if(gamepad1.b)
            power = 0.2;
        else if(gamepad1.a)
            power = 0.3;
        else if(gamepad1.x)
            power = 0.4;
        else if(gamepad1.dpad_up)
            power = 0.5;
        else if(gamepad1.dpad_right)
            power = 0.6;
        else if(gamepad1.dpad_down)
            power = 0.7;
        else if(gamepad1.dpad_left)
            power = 0.8;
        else if(gamepad1.right_bumper)
            power = 0.9;
        else if(gamepad1.left_bumper)
            power = 1;
        else
            power = 0;

        motor.setPower(power);
        telemetry.addData("Motor Power", power);
        telemetry.update();

    }
}
