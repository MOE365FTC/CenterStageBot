package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;

@Config
public class Outtake {
    private PIDController controller;
    Servo leftIris, rightIris, pitchServo, yawServo;
    DcMotor liftMotor, tiltMotor; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm


    //presets
    public static final double irisExpand = 0.2, irisContract = 0, intakePitch = 1, scorePitch = 0.68; //defaults
    public static final double liftPower = 0.7; //tuning needed
    public static final int pitchAutoThreshold = 150; //tuning needed
    public static final int tiltBase = 0, tiltTransfer = 300, tiltBoard = 700, liftBase = 0, liftLow = 200; //tuning needed
    double tiltPower;
    public TiltPositions currTiltPos = TiltPositions.DOWN;

    public int liftTarget = liftBase;


    public static final int MIN_TILT_TICKS = 200; //tune
    public static final int MAX_TILT_TICKS = 1000; //tune
    public static final int MAX_LIFT_TICKS = 1000; //tune


    //pidf rot arm
    //notes to tune: 1. find lowest f value that hold position, 2. find close p value, 3. find d and f, 4. loop step 2 and 3
    public static double p = 0.0075, i = 0.3, d = 0.0004, f = 0.07; //has slight problems on way down;
    public static int tiltTarget = tiltBase; //initialize to tiltBase

    //pitch servo parameters
    private final double tiltMotorTicksPerDegree= 2050.0/180.0;//needs calibration
    int pitchServoTotalDegrees = 180; //check this

    //yaw presets
    public static final double yawVertical = 0.0, yawHorizontal = 1.0;

    //intake state
    private Intake.ExtendPositions oldExtendState;

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
        liftMotor = hardwareMap.get(DcMotor.class, "armExtend");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        yawServo = hardwareMap.get(Servo.class, "yawServo");

        //iris setup
        leftIris.setPosition(irisExpand);
        rightIris.setPosition(irisExpand);

        //wrist setup
        pitchServo.setPosition(intakePitch);
        yawServo.setPosition(yawVertical);

        //tilt setup
        tiltMotor.setDirection(DcMotorSimple.Direction.REVERSE); //CHECK THIS
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setTargetPosition(0);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //tilt PID setup
        controller = new PIDController(p, i, d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TimerTask setup
        timer = new Timer("timer");
        delayedTiltToBase = new TimerTask() {
            @Override
            public void run() {
                tiltTarget = tiltBase;
            }
        };
    }

    public void actuate() {
        //iris
        if(currTiltPos == TiltPositions.DOWN) {
            leftIris.setPosition(irisContract);
            rightIris.setPosition(irisContract);
        } else {
            if(gamepad2.left_bumper)
                leftIris.setPosition(irisContract);
            else
                leftIris.setPosition(irisExpand);

            if(gamepad2.right_bumper)
                rightIris.setPosition(irisContract);
            else
                rightIris.setPosition(irisExpand);
        }

        //pitch servo
//        if (liftMotor.getCurrentPosition() > pitchAutoThreshold && !liftMotor.isBusy())
//            pitchServo.setPosition((120 - (tiltMotor.getCurrentPosition() / tiltMotorTicksPerDegree )) / pitchServoTotalDegrees);

        //manual lift
        if(liftMotor.getCurrentPosition() > MIN_TILT_TICKS) {
            if (-gamepad2.left_stick_y > 0.75 && liftMotor.getCurrentPosition() <= MAX_LIFT_TICKS - 100)
                liftTarget += 100;
            else if (-gamepad2.left_stick_y < -0.75 && liftMotor.getCurrentPosition() >= 100)
                liftTarget -= 100;
        }

        //manual tilt arm
        if(currTiltPos == TiltPositions.UP) { //to tilt out use presets and then fine tuning with manual
            if (-gamepad2.right_stick_y > 0.75 && tiltTarget <= MAX_TILT_TICKS - 75) {
                tiltTarget += 75;
            } else if (-gamepad2.right_stick_y < -0.75 && tiltTarget >= MIN_TILT_TICKS + 75)
                tiltTarget -= 75;
        }

        //tilt+lift presets
        if(gamepad2.dpad_up) {
            currTiltPos = TiltPositions.UP;
            tiltTarget = tiltBoard;
            liftTarget = liftLow;
        } else if (gamepad2.dpad_down) {
            currTiltPos = TiltPositions.DOWN;
            pitchServo.setPosition(intakePitch);
            yawServo.setPosition(yawVertical);
            liftTarget = liftLow;
            timer.schedule(delayedTiltToBase, 500);
        }

        if(Intake.currExtendPos == Intake.ExtendPositions.TRANSFER) {
            tiltTarget = tiltTransfer;
            currTiltPos = TiltPositions.UP;
        } else if(Intake.currExtendPos == Intake.ExtendPositions.BASE && oldExtendState == Intake.ExtendPositions.TRANSFER) {
            tiltTarget = tiltBase;
            currTiltPos = TiltPositions.DOWN;
        }
        oldExtendState = Intake.currExtendPos;

        if(tiltMotor.getCurrentPosition() > MIN_TILT_TICKS) {
            liftArm(liftTarget);
            pitchServo.setPosition(scorePitch);
            yawServo.setPosition(yawHorizontal);
        }

        tiltArm(tiltTarget);
    }

    private void tiltArm(int targetPos) {
        tiltMotor.setTargetPosition(targetPos);
        tiltMotor.setPower(liftPower);
    }

    private void liftArm(int targetPos) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(liftPower);
    }

    public void autonTilt(int target) { //use in teleop and auto by changing the tiltTarget variable (this will be automatically called at the end of loops)


//        controller.setPID(p, i, d);
//        int armPos = tiltMotor.getCurrentPosition();
//        double pid = controller.calculate(armPos,target);
//        double ff = Math.cos(Math.toRadians(target/tiltMotorTicksPerDegree)) * f;
//
//        tiltPower = pid + ff;
//        tiltMotor.setPower(tiltPower);
//
//        telemetry.addData("pos", armPos);
//        telemetry.addData("target", target);
//        telemetry.addData("tilt power", tiltPower);
//        telemetry.addData("pid", pid);
//        telemetry.update();
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

    public void telemetryOuttake() {
        telemetry.addData("Extension", liftMotor.getCurrentPosition());
        telemetry.addData("Tilt", tiltMotor.getCurrentPosition());
    }

    public enum autonLiftPositions {
        BASE,
        LOW
    }

    public enum TiltPositions {
        DOWN,
        UP
    }

}
