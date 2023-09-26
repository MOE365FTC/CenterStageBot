package org.firstinspires.ftc.teamcode.backup.backupHardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BackupClaw {
    Servo clawServo;
    Gamepad gamepad1;
    Gamepad gamepad2;
    HardwareMap hardwareMap;

    public double open = 0.2, closed = 0.83, center = 0.5;
    //teleop
    public BackupClaw(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        clawServo = hardwareMap.get(Servo.class, "CGS10");
    }

    public void actuate(){
        if(gamepad1.left_bumper || gamepad2.left_bumper){
            clawServo.setPosition(closed);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper){
            clawServo.setPosition(open);
        }
    }

    public void grab(){
        clawServo.setPosition(closed);
    }

    public void release(){
        clawServo.setPosition(open);
    }

    public double getClawTicks() {
        return clawServo.getPosition();
    }

    public void nudgeClaw (double stickY) {
        clawServo.setPosition(getClawTicks() + 0.1 * -Math.signum(stickY));
    }

    public void startClawPos(){
        clawServo.setPosition(open);
    }

    public void setPos(double pos){
        clawServo.setPosition(pos);
    }
}