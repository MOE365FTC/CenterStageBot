package org.firstinspires.ftc.teamcode.backup.backupHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BackupLift {
    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;

    public DcMotor liftMotor;
    public Servo forkServo;
    int high = 1975, mid = 1410, low = 853, floor = 0, stack5 = 450, stack4 = 400, stack3 = 300, stack2 = 200; //lift extension
    int maxLiftPos = 2200; //FIND THIS, TEMP VALUE FOR RIGHT NOW
    int manualIncrement = 90;
    double liftPower = 1.0;
    double autonLiftPower = 0.7;
    double forkUp = 1.0, forkDown = 0.22;
    public BackupLift(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        liftMotor = hardwareMap.get(DcMotor.class, "SLM10");
        forkServo = hardwareMap.get(Servo.class, "JSS11");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void actuate(){
        if(gamepad1.y){
            liftMotor.setTargetPosition(high);
            liftMotor.setPower(liftPower);
        } else if (gamepad1.x){
            liftMotor.setTargetPosition(mid);
            liftMotor.setPower(liftPower);
        } else if (gamepad1.b){
            liftMotor.setTargetPosition(low);
            liftMotor.setPower(liftPower);
        } else if (gamepad1.a) {
            liftMotor.setTargetPosition(floor);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.dpad_up){
            liftMotor.setTargetPosition(stack5);
            liftMotor.setPower(liftPower);
        }else if (gamepad2.dpad_right){
            liftMotor.setTargetPosition(stack4);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.dpad_left){
            liftMotor.setTargetPosition(stack3);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.dpad_down){
            liftMotor.setTargetPosition(stack2);
            liftMotor.setPower(liftPower);
        }
       if(liftMotor.getTargetPosition() < 100 && liftMotor.getCurrentPosition() < 300){
            liftMotor.setPower(0.0);
        }
    }

    public void autonActuate(autonLiftPos pos) {
        switch (pos) {
            case HIGH:
                forkServo.setPosition(forkDown);
                liftMotor.setTargetPosition(high);
                liftMotor.setPower(autonLiftPower);
                break;
            case MID:
                forkServo.setPosition(forkDown);
                liftMotor.setTargetPosition(mid);
                liftMotor.setPower(autonLiftPower);
                break;
            case LOW:
                forkServo.setPosition(forkUp);
                liftMotor.setTargetPosition(low);
                liftMotor.setPower(autonLiftPower);
                break;
            case FLOOR:
                forkServo.setPosition(forkUp);
                liftMotor.setTargetPosition(floor);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB5:
                forkServo.setPosition(forkUp);
                liftMotor.setTargetPosition(stack5);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB4:
                forkServo.setPosition(forkUp);
                liftMotor.setTargetPosition(stack4);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB3:
                forkServo.setPosition(forkUp);
                liftMotor.setTargetPosition(stack3);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB2:
                forkServo.setPosition(forkUp);
                liftMotor.setTargetPosition(stack2);
                liftMotor.setPower(autonLiftPower);
                break;
        }
    }

    public void forkActuate(){
        if(!lowered) {
            if (liftMotor.getCurrentPosition() < 1300) {
                forkServo.setPosition(forkUp);
            } else if (liftMotor.getCurrentPosition() > 1300 & liftMotor.getTargetPosition() > 1300) {
                forkServo.setPosition(forkDown);
            }
        }
    }

    public enum autonLiftPos {
        HIGH,
        MID,
        LOW,
        FLOOR,
        GRAB5,
        GRAB4,
        GRAB3,
        GRAB2,
        LOW_GRAB,
    }

    public int getLiftTicks() {
        return liftMotor.getCurrentPosition();
    }

    public void manualLift(){
        if(gamepad2.right_trigger > 0.1 && liftMotor.getTargetPosition() < maxLiftPos){
            liftMotor.setTargetPosition((liftMotor.getCurrentPosition() + manualIncrement));
            liftMotor.setPower(liftPower);
        } else if(gamepad2.left_trigger > 0.1 && liftMotor.getTargetPosition() > 0){
            liftMotor.setTargetPosition((liftMotor.getCurrentPosition() - manualIncrement));
            liftMotor.setPower(liftPower);
        }
    }

    boolean lowered = false;
    int pos;
    public void lowerLift(){
        if(!lowered){
            pos = liftMotor.getTargetPosition();
        }
        if(gamepad1.left_trigger > 0.3 && liftMotor.getCurrentPosition() > 500){
            liftMotor.setTargetPosition(pos-300);
            lowered = true;
        } else{
            liftMotor.setTargetPosition(pos);
            lowered = false;
        }
    }

    public void lowerAuton(double ticks){
        liftMotor.setTargetPosition(liftMotor.getTargetPosition() - (int) ticks);
        liftMotor.setPower(autonLiftPower);
    }

    public int getTargetPosition(){
        return liftMotor.getTargetPosition();
    }
}