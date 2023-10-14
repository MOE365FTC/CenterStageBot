package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.rr.trajectorysequence.sequencesegment.SequenceSegment;

@TeleOp
public class servoTest extends OpMode {
    ServoImplEx servo;
    Servo servo1;
    ServoControllerEx controller;
    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, "SR001");
        servo1 = hardwareMap.get(Servo.class, "SR002");
        controller = (ServoControllerEx) servo.getController();
    }



    @Override
    public void loop() {
        if (gamepad1.a){
            controller.setServoPwmDisable(servo.getPortNumber());
        }
        else if (gamepad1.b){
            servo.setPwmEnable();
            servo.setPosition(0.0);
            servo1.setPosition(0.0);
        } else if (gamepad1.x) {
            servo.setPwmEnable();
        } else if (gamepad1.y) {
            servo.setPosition(0.7);
        } else if (gamepad1.right_trigger > 0.3){
            servo1.setPosition(0.7);}
    }
}
