package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testPower extends OpMode {
    DcMotor test;

    @Override
    public void init() {
        test = hardwareMap.get(DcMotor.class, "LM");

        test.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (-gamepad2.right_stick_y > 0.75)
            test.setPower(0.5);
        else if (-gamepad2.right_stick_y < -0.75)
            test.setPower(-0.5);
        else
            test.setPower(0);
    }
}
