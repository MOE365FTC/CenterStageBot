package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DispenserDec17 {
    //DEVICES
    Servo leftIris, rightIris, pitchServo;
    DcMotor liftMotor;
    ColorSensor colorSensorLeft, colorSensorRight;

    //PRESETS
    double irisExpand = 0.2, irisContract = 0, intakePitch = 1, scorePitch = 0.68; //defaults
    int extendLow = 560, extendMid = 1350, extendFull = 2100, extendPreIntake = 200, extendIntake = 20;//lift presets
    double extendPower = 0.7;
    double LEFT_STICK_THRESHOLD = 0.75; // pulling the left joystick below this limit makes the intake dive to pick up
    int setPos = 0;

    //TOGGLES
    boolean leftClosed = false, rightClosed = false, scheduledOpen = false;
    boolean oldLeftBumper = false, oldRightBumper = false;

    Gamepad gamepad2;
    Telemetry telemetry;

    public DispenserDec17(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        liftMotor = hardwareMap.get(DcMotor.class, "LM");
//        colorSensorRight = hardwareMap.get(ColorSensor.class, "CSR");
//        colorSensorLeft = hardwareMap.get(ColorSensor.class, "CSL");

        leftIris.setPosition(irisExpand);
        rightIris.setPosition(irisExpand);
        pitchServo.setPosition(intakePitch);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(extendPreIntake);
        setPos = extendPreIntake;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate() {
        if(gamepad2.left_bumper)
            leftIris.setPosition(irisContract);
        else if(gamepad2.left_stick_y < LEFT_STICK_THRESHOLD)
            leftIris.setPosition(irisExpand);

        if(gamepad2.right_bumper)
            rightIris.setPosition(irisContract);
        else if(gamepad2.left_stick_y < LEFT_STICK_THRESHOLD)
            rightIris.setPosition(irisExpand);

        if(gamepad2.left_stick_y > LEFT_STICK_THRESHOLD){
            leftIris.setPosition(irisContract);
            rightIris.setPosition(irisContract);
            pitchServo.setPosition(intakePitch);
            setPos = extendIntake;
        }
        if(gamepad2.dpad_up) { //max height lift
            setPos = extendFull;
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.dpad_right) { //mid height lift
            setPos = extendMid;
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.dpad_left) { //low height lift
            setPos = extendLow;
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.dpad_down) { //intake height lift
//            scheduledOpen = true; //can't open irises instantly, must wait until lift has stopped
            setPos = extendPreIntake;
            pitchServo.setPosition(intakePitch);
        } else if(gamepad2.y) {
            setPos = liftMotor.getCurrentPosition() + 50;
        } else if(gamepad2.a) {
            setPos = liftMotor.getCurrentPosition() - 50;
        }
        //code segment to reopen irises once dispenser is back in intake position and lifts are at rest
//        if(scheduledOpen && !leftLift.isBusy() && !rightLift.isBusy()) {
//            leftIris.setPosition(irisOpen);
//            rightIris.setPosition(irisOpen);
//            scheduledOpen = false;
//        }

//        telemetryPixelColors();
        extendLift(setPos);
    }

    private void extendLift(int targetPos) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(extendPower);
    }


    public void autonIris(boolean expand) {
      leftIris.setPosition(expand ? irisExpand : irisContract);
      rightIris.setPosition(expand ? irisExpand : irisContract);
    }

    public void autonLeftIris(boolean expand) {
        leftIris.setPosition(expand ? irisExpand : irisContract);
    }

    public void autonRightIris(boolean open) {
        rightIris.setPosition(open ? irisExpand : irisContract);
    }

    public void autonLift(autonLiftPositions liftPos) {
        switch (liftPos) {
            case PRE_INTAKE:
                pitchServo.setPosition(intakePitch);
                extendLift(extendPreIntake);
                break;
            case LOW:
                pitchServo.setPosition(scorePitch);
                extendLift(extendLow);
                break;
            case MID:
                pitchServo.setPosition(scorePitch);
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

    public void telemetryLiftPosition() {
        telemetry.addData("Lift Height", liftMotor.getCurrentPosition());
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
        PRE_INTAKE,
        LOW,
        MID,
    }
}
