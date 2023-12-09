package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hanging {
    DcMotor leftHangMotor, rightHangMotor;
    Gamepad gamepad1;

    double hangPower = 0.75;

    public Hanging(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;

        leftHangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        leftHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightHangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        rightHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void actuate(){
        if (gamepad1.dpad_up) {
            leftHangMotor.setTargetPosition(900);
            rightHangMotor.setTargetPosition(900);
            leftHangMotor.setPower(hangPower);
            rightHangMotor.setPower(hangPower);
        }
    }
}


