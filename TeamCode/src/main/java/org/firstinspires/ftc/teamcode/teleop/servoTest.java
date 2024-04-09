package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(group = "test")
public class servoTest extends OpMode {
    Servo grabLeft, grabRight;
    Servo pitchServo, boxGateMain, boxGateAux;
    Servo droneLauncher;

    public static double pos = 0.0;

    @Override
    public void init() {
       grabLeft = hardwareMap.get(Servo.class, "grabLeft");
       grabRight = hardwareMap.get(Servo.class, "grabRight");

       pitchServo = hardwareMap.get(Servo.class, "pitchServo");
       boxGateMain = hardwareMap.get(Servo.class, "boxGateMain");
       boxGateAux = hardwareMap.get(Servo.class, "boxGateAux");

       droneLauncher = hardwareMap.get(Servo.class, "droneServo");

       grabLeft.setPosition(0);
       grabRight.setPosition(1);
    }



    @Override
    public void loop() {
        if(gamepad1.a) {
            boxGateAux.setPosition(pos);
            boxGateMain.setPosition(pos);
        }
    }
}
