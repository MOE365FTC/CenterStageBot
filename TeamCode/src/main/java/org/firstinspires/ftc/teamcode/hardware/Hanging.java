package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hanging {
    DcMotorEx leftHangMotor, rightHangMotor;
    Servo hangLockServo;
    Gamepad gamepad2;

    double hangPower = 1;
    double hookLockedPos = 0.95, hookReleasedPos = 0.7;

    public Hanging(HardwareMap hardwareMap, Gamepad gamepad2){
        this.gamepad2 = gamepad2;

        leftHangMotor = hardwareMap.get(DcMotorEx.class, "hangMotorL");
        leftHangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightHangMotor = hardwareMap.get(DcMotorEx.class, "hangMotorR");
        rightHangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftHangMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightHangMotor.setDirection(DcMotorEx.Direction.REVERSE);

        hangLockServo = hardwareMap.get(Servo.class, "hangLock");
        hangLockServo.setPosition(hookLockedPos);
    }
    public void actuate(){
        if (gamepad2.y) {
            leftHangMotor.setPower(hangPower);
            rightHangMotor.setPower(hangPower);
        }
        else if (gamepad2.a) {
            leftHangMotor.setPower(-hangPower);
            rightHangMotor.setPower(-hangPower);
        }
        else{
            leftHangMotor.setPower(0);
            rightHangMotor.setPower(0);
        }

        if(gamepad2.b)
            hangLockServo.setPosition(hookReleasedPos);
    }
}


