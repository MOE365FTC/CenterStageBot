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
    int extendLow = 510, extendMid = 1300, extendHigh = 2050, extendPreIntake = 150, extendIntake = 20; //lift presets
    double extendPower = 0.7;
    double LEFT_STICK_THRESHOLD = 0.75; // pulling the left joystick below this limit makes the intake dive to pick up
    int setPos = 0; //the target position the lift is set to
    int stage = 0; //the state the lift is in (0 - intake, 1 - preIntake, 2 - low, 3 - med, 4 - high)
    boolean stageLock = false; //stageLock prevents holding the joystick up/down in the SmartLift section (Lines 56-92)
    boolean oldX;

    Gamepad gamepad2;
    Telemetry telemetry;

    public DispenserDec17(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        liftMotor = hardwareMap.get(DcMotor.class, "LM");

        leftIris.setPosition(irisExpand);
        rightIris.setPosition(irisExpand);
        pitchServo.setPosition(intakePitch);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(extendPreIntake); //init set to pre intake position (power is not provided until run)
        setPos = extendPreIntake;
        stage = 1;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate() {
        //SmartLift system
        if(-gamepad2.left_stick_y < -0.75 && !stageLock) { //flick joystick down to lower by one stage (can't hold)
            if(stage!=0) stage--;
            stageLock = true;
        } else if(-gamepad2.left_stick_y > 0.75 && !stageLock){ //flick joystick up to raise by one stage (can't hold)
            if(stage!=4) stage++;
            stageLock = true;
        } else if(-gamepad2.left_stick_y < 0.10 && -gamepad2.left_stick_y > -0.10) { //turns off stage lock when joystick returns to center
            stageLock = false;
        }
        if(-gamepad2.left_stick_y > 0.75 || -gamepad2.left_stick_y < -0.75) { //if actively switching stages, move lift
            switch(stage) {
                case 0:
                    leftIris.setPosition(irisContract);
                    rightIris.setPosition(irisContract);
                    pitchServo.setPosition(intakePitch);
                    setPos = extendIntake;
                    break;
                case 1:
                    pitchServo.setPosition(intakePitch);
                    setPos = extendPreIntake;
                    break;
                case 2:
                    pitchServo.setPosition(scorePitch);
                    setPos = extendLow;
                    break;
                case 3:
                    pitchServo.setPosition(scorePitch);
                    setPos = extendMid;
                    break;
                case 4:
                    pitchServo.setPosition(scorePitch);
                    setPos = extendHigh;
                    break;
                default:
                    setPos = extendPreIntake;
            }
        }

        //IRIS CONTROLS
        if(gamepad2.left_bumper)
            leftIris.setPosition(irisContract);
        else if(gamepad2.left_stick_y < LEFT_STICK_THRESHOLD) //expanded by default (down on stick is a positive value)
            leftIris.setPosition(irisExpand);

        if(gamepad2.right_bumper)
            rightIris.setPosition(irisContract);
        else if(gamepad2.left_stick_y < LEFT_STICK_THRESHOLD) //expanded by default (down on stick is a positive value)
            rightIris.setPosition(irisExpand);

        //LIFT PRESETS
        if(gamepad2.dpad_up) { //max height lift
            setPos = extendHigh;
            stage = 4;
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.dpad_right) { //mid height lift
            setPos = extendMid;
            stage = 3;
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.dpad_left) { //low height lift
            setPos = extendLow;
            stage = 2;
            pitchServo.setPosition(scorePitch);
        } else if(gamepad2.dpad_down) { //intake height lift
            setPos = extendPreIntake;
            stage = 1;
            pitchServo.setPosition(intakePitch);
        } else if(gamepad2.y) {
            stage = -1;
            setPos = liftMotor.getCurrentPosition() + 50;
        } else if(gamepad2.a) {
            stage = -1;
            setPos = liftMotor.getCurrentPosition() - 50;
        }

        extendLift(setPos); //gives power to actually move the lift to target position
        resetLiftEncoder(); //runs another actuate function to allow the ability to reset lift encoder
    }

    private void extendLift(int targetPos) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(extendPower);
    }


    public void autonIris(boolean expand) { //auton method for iris control
        leftIris.setPosition(expand ? irisExpand : irisContract);
        rightIris.setPosition(expand ? irisExpand : irisContract);
    }

    public void autonLeftIris(boolean expand) { //individual auton iris control
        leftIris.setPosition(expand ? irisExpand : irisContract);
    }

    public void autonRightIris(boolean open) { //individual auton iris control
        rightIris.setPosition(open ? irisExpand : irisContract);
    }

    public void autonLift(autonLiftPositions liftPos) { //auton lift control (pass in an enum value)
        switch (liftPos) {
            case AUTON_INTAKE:
                pitchServo.setPosition(intakePitch);
                extendLift(extendPreIntake - 100);
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

    public void resetLiftEncoder() {
        if(gamepad2.x && !oldX) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
            stage = 0;
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        oldX = gamepad2.x;
    }

    private boolean colorInRange(ColorSensor cs, int lowR, int highR, int lowG, int highG, int lowB, int highB) {
        return lowR < cs.red() && cs.red() < highR && lowG < cs.green() && cs.green() < highG && lowB < cs.blue() && cs.blue() < highB;
    }

    public enum autonLiftPositions {
        AUTON_INTAKE,
        PRE_INTAKE,
        LOW,
        MID,
    }
}