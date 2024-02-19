package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;

@Config
public class Outtake {
    private PIDController controller;
    Servo leftIris, rightIris, pitchServo;
    CRServo gateLeft, gateRight;
    DcMotor liftMotor, tiltMotor, intakeMotor, intakeSlides; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm
    DigitalChannel gateLeftSwitch, gateRightSwitch;


    //presets
    public static final double irisExpand = 0.2, irisContract = 0, intakePitch = 1, scorePitch = 0.68; //defaults
    public static final double liftPower = 0.7, intakeSlidesPower = 0.7, intakeWheelPower = 0.7; //tuning needed
    public static final int pitchAutoThreshold = 150; //tuning needed
    public static final int intakeSlidesBase = 0, intakeSlidesOut = 100, tiltBase = 0, tiltBoard = 700, liftBase = 0, liftLow = 200; //tuning needed
    double tiltPower;

    public static final int MAX_TILT_TICKS = 1000; //tune
    public static final int MAX_EXTEND_TICKS = 1000; //tune
    public static final int MAX_LIFT_TICKS = 1000; //tune

    double gatePower = 0.7;
    boolean oldGateLeftClicked = true, oldGateRightClicked = true;
    boolean runGateL = false, runGateR = false;


    //pidf rot arm
    //notes to tune: 1. find lowest f value that hold position, 2. find close p value, 3. find d and f, 4. loop step 2 and 3
    public static double p = 0.0075, i = 0.3, d = 0.0004, f = 0.07; //has slight problems on way down;
    public static int tiltTarget = tiltBase; //initialize to tiltBase

    //pitch servo parameters
    private final double tiltMotorTicksPerDegree= 2050.0/180.0;//needs calibration
    int pitchServoTotalDegrees = 180; //check this

    //timers
    TimerTask delayedExtendToLow, delayedTiltToBase;
    Timer timer;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    public Outtake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        //hardware setup
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        gateLeft = hardwareMap.get(CRServo.class, "gateLeft");
        gateRight = hardwareMap.get(CRServo.class, "gateRight");
        liftMotor = hardwareMap.get(DcMotor.class, "armExtend");
        tiltMotor = hardwareMap.get(DcMotor.class, "LM");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");

        //gate setup
        gateLeftSwitch.setMode(DigitalChannel.Mode.INPUT);
        gateRightSwitch.setMode(DigitalChannel.Mode.INPUT);

        //iris setup
        leftIris.setPosition(irisExpand);
        rightIris.setPosition(irisExpand);

        //pitch setup
        pitchServo.setPosition(intakePitch);

        //tilt setup
        tiltMotor.setDirection(DcMotorSimple.Direction.REVERSE); //CHECK THIS
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lift
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //extend
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //intake motor
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //tilt PID setup
        controller = new PIDController(p, i, d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TimerTask setup
        timer = new Timer("timer");
        delayedExtendToLow = new TimerTask() {
            @Override
            public void run() {
                liftArm(liftLow);
            }
        };
        delayedTiltToBase = new TimerTask() {
            @Override
            public void run() {
                tiltTarget = tiltBase;
            }
        };
    }

    public void actuate() {
        //iris
        if(gamepad2.left_bumper)
            leftIris.setPosition(irisContract);
        else
            leftIris.setPosition(irisExpand);

        if(gamepad2.right_bumper)
            rightIris.setPosition(irisContract);
        else
            rightIris.setPosition(irisExpand);

        //pitch servo
//        if (liftMotor.getCurrentPosition() > pitchAutoThreshold && !liftMotor.isBusy())
//            pitchServo.setPosition((120 - (tiltMotor.getCurrentPosition() / tiltMotorTicksPerDegree )) / pitchServoTotalDegrees);

        //manual lift
        if (-gamepad2.left_stick_y > 0.75 && liftMotor.getCurrentPosition() <= MAX_LIFT_TICKS - 100)
            liftArm(liftMotor.getCurrentPosition() + 100);
        else if (-gamepad2.left_stick_y < -0.75 && liftMotor.getCurrentPosition() >= 100)
            liftArm(liftMotor.getCurrentPosition() - 100);

        //manual tilt arm
        if (-gamepad2.right_stick_y > 0.75 && tiltTarget <= MAX_TILT_TICKS - 75)
            tiltTarget += 75;
        else if (-gamepad2.right_stick_y < -0.75 && tiltTarget >= 75)
            tiltTarget -= 75;

        //manual intake slides
        if (gamepad1.right_trigger > 0.75 && intakeSlides.getCurrentPosition() <= MAX_EXTEND_TICKS - 50)
            extendIntake(intakeSlides.getCurrentPosition()+50);
        else if (gamepad1.left_trigger > 0.75 && intakeSlides.getCurrentPosition() >= 50)
            extendIntake(intakeSlides.getCurrentPosition()-50);
        //preset intake slides
        else if (gamepad1.b)
            extendIntake(intakeSlidesBase);
        else if (gamepad1.x)
            extendIntake(intakeSlidesOut);

        //tilt+lift presets
        if(gamepad2.dpad_up) {
            tiltTarget = tiltBoard;
            timer.schedule(delayedExtendToLow, 500);
        } else if (gamepad2.dpad_down) {
            liftArm(liftBase);
            timer.schedule(delayedTiltToBase, 500);
        }

        //intake motor
        if (gamepad1.a)
            intakeMotor.setPower(intakeWheelPower);
        else
            intakeMotor.setPower(0);

        //pidf loop arm
        tiltPID(tiltTarget);

    }

    private void liftArm(int targetPos) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(liftPower);
    }

    private void extendIntake(int targetPos) {
        intakeSlides.setTargetPosition(targetPos);
        intakeSlides.setPower(intakeSlidesPower);
    }

    public void tiltPID(int target) { //use in teleop and auto by changing the tiltTarget variable (this will be automatically called at the end of loops)
        controller.setPID(p, i, d);
        int armPos = tiltMotor.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target/tiltMotorTicksPerDegree)) * f;

        tiltPower = pid + ff;
        tiltMotor.setPower(tiltPower);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("tilt power", tiltPower);
        telemetry.addData("pid", pid);
        telemetry.update();
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

    public void autonIntakeSlides(autonExtendPositions pos) {
        switch(pos) {
            case BASE:
                extendIntake(intakeSlidesBase);
                break;
            case EXTENDED_FULL:
                extendIntake(intakeSlidesOut);
        }
    }

    public void autonLift(autonLiftPositions pos) {
        switch(pos) {
            case LOW:
                liftArm(liftLow);
            case BASE:
                liftArm(liftBase);
            default:
                liftArm(liftBase);
        }
    }

    public void runGates() { //called once in auton to update runGate variables so that on the next loop of updateGates() the gates will spin once
        runGateL = true;
        runGateR = true;
    }

    public void updateGates(){ //needs to be looped in auton
        if(runGateL) {
            gateLeft.setPower(gatePower);
            if(gateLeftSwitch.getState() && !oldGateLeftClicked) {
                gateLeft.setPower(0);
                runGateL = false;
            }
            oldGateLeftClicked = gateLeftSwitch.getState();

        }
        if(runGateR) {
            gateRight.setPower(gatePower);
            if (gateRightSwitch.getState() && !oldGateRightClicked) {
                gateRight.setPower(0);
                runGateR = false;
            }
            oldGateRightClicked = gateRightSwitch.getState();
        }
    }

    public void telemetryOuttake() {
        telemetry.addData("Extension", liftMotor.getCurrentPosition());
        telemetry.addData("Tilt", tiltMotor.getCurrentPosition());
        telemetry.addData("intakeSlides", intakeSlides.getCurrentPosition());
    }

    public enum autonLiftPositions {
        BASE,
        LOW
    }

    public enum autonExtendPositions {
        EXTENDED_FULL,
        BASE
    }
}
