package org.firstinspires.ftc.teamcode.rr.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive extends OpMode {
    DcMotor leftFront, leftRear, rightFront, rightRear;
    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "FLM");
        leftRear = hardwareMap.get(DcMotorEx.class, "BLM");
        rightFront = hardwareMap.get(DcMotorEx.class, "FRM");
        rightRear = hardwareMap.get(DcMotorEx.class, "BRM");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        leftFront.setPower(y + x + rx);
        leftRear.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightRear.setPower(y + x - rx);
    }
}
