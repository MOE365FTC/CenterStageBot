package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hanging {
    DcMotorEx leftHangMotor, rightHangMotor;
    Servo PTO;
    Gamepad gamepad2;

    double hangPower = 1;
    double lockedPos = 0.9, releasedPos = 0.45;

    public Hanging(HardwareMap hardwareMap, Gamepad gamepad2){
        PTO = hardwareMap.get(Servo.class, "PTO");
        PTO.setPosition(lockedPos);
    }
    public void actuate(){
        if (gamepad2.right_trigger > 0.3) {
            leftHangMotor.setPower(hangPower);
            rightHangMotor.setPower(hangPower);
        }
        else if (gamepad2.left_trigger > 0.3) {
            leftHangMotor.setPower(-hangPower);
            rightHangMotor.setPower(-hangPower);
        }
        else{
            leftHangMotor.setPower(0);
            rightHangMotor.setPower(0);
        }

        if(gamepad2.b)
            PTO.setPosition(releasedPos);
    }
}


