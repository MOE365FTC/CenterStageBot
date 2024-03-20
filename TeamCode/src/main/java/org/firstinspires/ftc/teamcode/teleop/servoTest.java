package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class servoTest extends OpMode {
    ServoImplEx servo;
    Servo servo1;
    ServoControllerEx controller;
    boolean disableServo = false;
    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, "SR001");
        servo1 = hardwareMap.get(Servo.class, "SR002");
    }



    @Override
    public void loop() {
        if (gamepad1.a){
            disableServo = true;
        }
        else if (gamepad1.b){
            if(!disableServo){
                servo.setPosition(0.0);
            }else{
                servo.setPwmDisable();
            }
            servo1.setPosition(0.0);
        } else if (gamepad1.x) {
            disableServo = false;
            servo.setPwmEnable();
        } else if (gamepad1.y) {
            if(!disableServo)
                servo.setPosition(0.7);
            else
                servo.setPwmDisable();
        } else if (gamepad1.right_trigger > 0.3){
            servo1.setPosition(0.7);
        }
    }
}
