package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hanging {
    DcMotor slideLeft, slideRight;
    Gamepad gamepad1;

    public Hanging(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;

        slideLeft = hardwareMap.get(DcMotor.class, "SLL");
        slideRight = hardwareMap.get(DcMotor.class, "SLR");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void actuate(){
        if (gamepad1.dpad_up) {
            slideLeft.setTargetPosition(900);
            slideRight.setTargetPosition(900);

            slideLeft.setPower(0.75);
            slideRight.setPower(0.75);
        }
    }
}


