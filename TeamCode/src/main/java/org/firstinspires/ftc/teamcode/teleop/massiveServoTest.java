package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "test")
@Config
public class massiveServoTest extends OpMode {
    Servo transferBeltServo, leftIris, rightIris, PTO, pitch, yaw, drone;
    CRServo grabRight, grabLeft;
    public static double setPos;
    @Override
    public void init() {
        transferBeltServo = hardwareMap.get(Servo.class, "transferBeltServo");
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        PTO = hardwareMap.get(Servo.class, "PTO");
        pitch = hardwareMap.get(Servo.class, "pitchServo");
        yaw = hardwareMap.get(Servo.class, "yawServo");
        drone = hardwareMap.get(Servo.class, "droneServo");
        grabLeft = hardwareMap.get(CRServo.class, "grabLeft");
        grabRight = hardwareMap.get(CRServo.class, "grabRight");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            transferBeltServo.setPosition(setPos);
            leftIris.setPosition(setPos);
            rightIris.setPosition(setPos);
            PTO.setPosition(setPos);
            pitch.setPosition(setPos);
            yaw.setPosition(setPos);
            drone.setPosition(setPos);
            grabRight.setPower(setPos);
            grabLeft.setPower(setPos);
        }
    }
}
