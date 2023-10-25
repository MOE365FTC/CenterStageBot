package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Dispenser {
    Servo leftIris, rightIris, rollServo, pitchServo;
    DcMotor leftLift, rightLift;
    ColorSensor colorSensorLeft, colorSensorRight;
    double irisOpen = 0.0, irisClose = 1.0, defaultPitch = 0.5, straightRoll = 0.5; //defaults
    double leftPitch = 0.0, rightPitch = 1.0, bottomRoll = 0.0; //servo presets
    int liftMid = 400, liftHigh = 800, liftDefault = 0;//lift presets
    double liftPower = 0.7;
    boolean leftClosed = false, rightClosed = false;

    Gamepad gamepad2;

    public Dispenser(HardwareMap hardwareMap, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        rollServo = hardwareMap.get(Servo.class, "roll");
        pitchServo = hardwareMap.get(Servo.class, "pitch");
        leftLift = hardwareMap.get(DcMotor.class, "LLM");
        rightLift = hardwareMap.get(DcMotor.class, "RLM");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "CSR");
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "CSL");
        leftIris.setPosition(irisOpen);
        rightIris.setPosition(irisOpen);
        rollServo.setPosition(straightRoll);
        pitchServo.setPosition(defaultPitch);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate() {
        /* color sensor auto iris close

        if the color sensor shows a pixel in iris, close iris and change boolean value

         */

        if(gamepad2.left_bumper && leftClosed) { //left iris toggle
            leftIris.setPosition(irisOpen);
            leftClosed = false;
        } else if(gamepad2.left_bumper) {
            leftIris.setPosition(irisClose);
            leftClosed = true;
        }
        if(gamepad2.right_bumper && rightClosed) { //right iris toggle
            rightIris.setPosition(irisOpen);
            rightClosed = false;
        } else if(gamepad2.right_bumper) {
            rightIris.setPosition(irisClose);
            rightClosed = true;
        }
        if(gamepad2.dpad_left) { //pitch control
            pitchServo.setPosition(leftPitch);
        } else if(gamepad2.dpad_up) {
            pitchServo.setPosition(defaultPitch);
        } else if(gamepad2.dpad_right) {
            pitchServo.setPosition(rightPitch);
        }
        if(gamepad2.x) { //lift control
            lift(liftMid);
            rollServo.setPosition(straightRoll);
        } else if(gamepad2.y) {
            lift(liftHigh);
            rollServo.setPosition(straightRoll);
        } else  if(gamepad2.a) {
            pitchServo.setPosition(defaultPitch);
            rollServo.setPosition(bottomRoll);
            lift(liftDefault);
        }
    }

    private void lift(int targetPos) {
        leftLift.setTargetPosition(targetPos);
        rightLift.setTargetPosition(targetPos);
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void autonIris(boolean open) {
      leftIris.setPosition(open ? irisOpen : irisClose);
      rightIris.setPosition(open ? irisOpen : irisClose);
    }

    public void autonLift(autonLiftPositions liftPos) {
        switch (liftPos) {
            case DEFAULT:
                pitchServo.setPosition(defaultPitch);
                rollServo.setPosition(straightRoll);
                lift(liftDefault);
                break;
            case MID:
                lift(liftMid);
                rollServo.setPosition(straightRoll);
                break;
            case HIGH:
                lift(liftHigh);
                rollServo.setPosition(straightRoll);
                break;
        }
    }

//    public boolean hasPlaced() {
//        //return true if color sensors detect nothing and it is in the lift is in the default (down) position
//    }

    public enum autonLiftPositions {
        HIGH,
        MID,
        DEFAULT
    }
}
