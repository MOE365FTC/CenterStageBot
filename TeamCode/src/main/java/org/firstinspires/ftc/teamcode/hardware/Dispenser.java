package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Deprecated
public class Dispenser {
    //DEVICES
    Servo leftIris, rightIris, pitchServo;
    DcMotor leftLift, rightLift, tiltMotor;
    ColorSensor colorSensorLeft, colorSensorRight;

    //PRESETS
    double irisOpen = 0.0, irisClose = 1.0, intakePitch = 0.5, scorePitch = 0.0; //defaults
    int extendMid = 400, extendFull = 800, extendIntake = 100;//lift presets
    int tiltIntake = 0, tiltScore = 800;
    double extendPower = 0.7;
    double tiltPower = 0.8;

    //TOGGLES
    boolean leftClosed = false, rightClosed = false, scheduledOpen = false;
    boolean oldLeftBumper = false, oldRightBumper = false;

    Gamepad gamepad2;
    Telemetry telemetry;

    public Dispenser(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        leftLift = hardwareMap.get(DcMotor.class, "LLM");
        rightLift = hardwareMap.get(DcMotor.class, "RLM");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "CSR");
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "CSL");

        leftIris.setPosition(irisOpen);
        rightIris.setPosition(irisOpen);
        pitchServo.setPosition(intakePitch);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);
        tiltMotor.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate() {
        if(gamepad2.left_bumper && !oldLeftBumper) { //left iris toggle
            if(leftClosed) {
                leftIris.setPosition(irisOpen);
                leftClosed = false;
            } else {
                leftIris.setPosition(irisClose);
                leftClosed = true;
            }
        }
        oldLeftBumper = gamepad2.left_bumper;

        if(gamepad2.right_bumper && !oldRightBumper) { //right iris toggle
            if(rightClosed) {
                leftIris.setPosition(irisOpen);
                rightClosed = false;
            } else {
                leftIris.setPosition(irisClose);
                rightClosed = true;
            }
        }
        oldRightBumper = gamepad2.right_bumper;

        if(gamepad2.y) { //max height lift
            leftIris.setPosition(irisClose);
            rightIris.setPosition(irisClose);
            tiltLift(tiltScore);
            extendLift(extendFull);
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.x) { //mid height lift
            leftIris.setPosition(irisClose);
            rightIris.setPosition(irisClose);
            tiltLift(tiltScore);
            extendLift(extendMid);
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.b) { //low height lift
            leftIris.setPosition(irisClose);
            rightIris.setPosition(irisClose);
            tiltLift(tiltScore); //no extension on low since the base arm itself is high enough
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.a) { //intake height lift
//            scheduledOpen = true; //can't open irises instantly, must wait until lift has stopped
            tiltLift(tiltIntake);
            extendLift(extendIntake);
            pitchServo.setPosition(intakePitch);
        }
        //code segment to reopen irises once dispenser is back in intake position and lifts are at rest
//        if(scheduledOpen && !leftLift.isBusy() && !rightLift.isBusy()) {
//            leftIris.setPosition(irisOpen);
//            rightIris.setPosition(irisOpen);
//            scheduledOpen = false;
//        }

        telemetryPixelColors();
    }

    private void extendLift(int targetPos) {
        leftLift.setTargetPosition(targetPos);
        rightLift.setTargetPosition(targetPos);
        leftLift.setPower(extendPower);
        rightLift.setPower(extendPower);
    }

    private void tiltLift(int targetPos) {
        tiltMotor.setTargetPosition(targetPos);
        tiltMotor.setPower(tiltPower);
    }

    public void autonIris(boolean open) {
      leftIris.setPosition(open ? irisOpen : irisClose);
      rightIris.setPosition(open ? irisOpen : irisClose);
    }

    public void autonLift(autonLiftPositions liftPos) {
        switch (liftPos) {
            case INTAKE:
                pitchServo.setPosition(intakePitch);
                tiltLift(tiltIntake);
                extendLift(extendIntake);
                break;
            case LOW:
                pitchServo.setPosition(scorePitch);
                tiltLift(tiltScore);
                break;
            case MID:
                pitchServo.setPosition(scorePitch);
                tiltLift(tiltScore);
                extendLift(extendMid);
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
        MID,
        LOW,
        INTAKE,
    }
}
