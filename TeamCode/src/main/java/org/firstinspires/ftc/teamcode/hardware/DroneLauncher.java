package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneLauncher {
    Servo launchServo;
    Gamepad gamepad1;

    double holdPosition = 0.0, launchPosition = 0.4;

    public DroneLauncher(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;
        launchServo = hardwareMap.get(Servo.class, "droneServo");
        launchServo.setPosition(holdPosition);
    }
    public void actuate(){if (gamepad1.dpad_right){launchServo.setPosition(launchPosition);}}
}
