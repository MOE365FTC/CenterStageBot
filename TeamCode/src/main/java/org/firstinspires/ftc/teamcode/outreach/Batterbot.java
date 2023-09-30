package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Batterbot extends OpMode {
    DcMotor frontLeft, backLeft, frontRight, backRight;
    CRServo batter;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        backLeft = hardwareMap.get(DcMotor.class, "BLM");
        frontRight = hardwareMap.get(DcMotor.class, "FRM");
        backRight = hardwareMap.get(DcMotor.class, "BRM");
        batter = hardwareMap.get(CRServo.class, "BATTER");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        backLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
        backRight.setPower(-gamepad1.right_stick_y);
        if(gamepad1.right_trigger > 0.3 ) {
            batter.setPower(0.7);
        } else if(gamepad1.left_trigger > 0.3) {
            batter.setPower(-0.7);
        } else {
            batter.setPower(0.0);
        }
    }
}
