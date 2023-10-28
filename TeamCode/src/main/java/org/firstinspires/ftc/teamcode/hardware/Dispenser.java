package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dispenser {
    Servo leftIris, rightIris, rollServo, pitchServo;
    DcMotor leftLift, rightLift;
    ColorSensor colorSensorLeft, colorSensorRight;
    double irisOpen = 0.0, irisClose = 1.0, defaultPitch = 0.5, straightRoll = 0.5; //defaults
    double leftPitch = 0.0, rightPitch = 1.0, bottomRoll = 0.0; //servo presets
    int liftMid = 400, liftHigh = 800, liftDefault = 0;//lift presets
    double liftPower = 0.7;
    boolean leftClosed = false, rightClosed = false, scheduledOpen = false;

    Gamepad gamepad2;
    Telemetry telemetry;

    public Dispenser(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
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
            leftIris.setPosition(irisClose);
            rightIris.setPosition(irisClose);
            lift(liftMid);
            rollServo.setPosition(straightRoll);
        } else if(gamepad2.y) {
            leftIris.setPosition(irisClose);
            rightIris.setPosition(irisClose);
            lift(liftHigh);
            rollServo.setPosition(straightRoll);
        } else  if(gamepad2.a) {
            scheduledOpen = true; //can't open irises instantly, must wait until lift has stopped
            pitchServo.setPosition(defaultPitch);
            rollServo.setPosition(bottomRoll);
            lift(liftDefault);
        }
        //code segment to reopen irises once dispenser is back in intake position and lifts are at rest
        if(scheduledOpen && !leftLift.isBusy() && !rightLift.isBusy()) {
            leftIris.setPosition(irisOpen);
            rightIris.setPosition(irisOpen);
            scheduledOpen = false;
        }

        telemetryPixelColors();
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

    public boolean leftIrisIsEmpty() {
        return detectPixelColors(colorSensorLeft).equals("None");
    }

    public boolean rightIrisIsEmpty() {
        return detectPixelColors(colorSensorRight).equals("None");
    }

    private void telemetryPixelColors() {
        telemetry.addData("Left Pixel", detectPixelColors(colorSensorLeft));
        telemetry.addData("Right Pixel", detectPixelColors(colorSensorRight));
    }

    private String detectPixelColors(ColorSensor cs) {
        if(colorInRange(cs, 500,1000,700, 1300, 50, 500)) {
            return "Yellow";
        } else if(colorInRange(cs, 0,500,350,950,0,500)) {
            return "Green";
        } else if(colorInRange(cs, 200,800,350,950,500,1000)) {
            return "Purple";
        } else if(colorInRange(cs, 1000,1600, 2000,2500,1200,1700)) {
            return "White";
        } else {
            return "None";
        }
    }

    private boolean colorInRange(ColorSensor cs, int lowR, int highR, int lowG, int highG, int lowB, int highB) {
        return lowR < cs.red() && cs.red() < highR && lowG < cs.green() && cs.green() < highG && lowB < cs.blue() && cs.blue() < highB;
    }

    public enum autonLiftPositions {
        HIGH,
        MID,
        DEFAULT
    }
}
