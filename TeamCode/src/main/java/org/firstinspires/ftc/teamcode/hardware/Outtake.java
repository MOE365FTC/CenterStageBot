package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    Servo leftIris, rightIris, pitchServo;
    DcMotor extensionMotor, tiltMotor, intakeMotor, intakeSlides; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm

    //presets
    public static final double irisExpand = 0.2, irisContract = 0, intakePitch = 1, scorePitch = 0.68; //defaults
    public static final double extendPower = 0.7; //tuning needed
    public static double tiltPower = 0.7; //tuning needed
    public static final double intakeSlidesPower = 0.7; //tuning needed
    public static final double intakePower = 0.7; //tuning needed
    public static final int pitchAutoThreshold = 150; //tuning needed
    public static final int intakeSlidesBase = 0; //tuning needed
    public static final int tiltBase = 0; //needs tuning
    public static final int tiltBoard = 700; //needs tuning
    public static final int extendBase = 0, extendLow = 200; //lift presets; needs tuning
    //pidf rot arm
    public static double p = 0.01, i = 0.1, d = 0.001, f = 0.1; //has slight problems on way down;
    public static int tiltTarget = tiltBase;

    private final double tiltMotorTicksPerDegree=2050/180;//needs calibration
    int pitchServoTotalDegrees = 180;//



    //status
    boolean leftClosed = false, rightClosed = false, scheduledOpen = false;
    boolean oldLeftBumper = false, oldRightBumper = false;
    boolean stageLock = false; //status if locked onto stage

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    TimerTask delayedArm, delayedTilt;
    Timer timer;
    public Outtake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        extensionMotor = hardwareMap.get(DcMotor.class, "extend");
        tiltMotor = hardwareMap.get(DcMotor.class, "LM");
        intakeMotor = hardwareMap.get(DcMotor.class, "IM");
        intakeSlides = hardwareMap.get(DcMotor.class, "IS");

        leftIris.setPosition(irisExpand);
        rightIris.setPosition(irisExpand);

        tiltMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pitchServo.setPosition(intakePitch);

        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        controller = new PIDController(p, i, d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new Timer("timer");
        delayedArm = new TimerTask() {
            @Override
            public void run() {
                extendArm(extendLow);
            }
        };

        delayedTilt = new TimerTask() {
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

        if (extensionMotor.getCurrentPosition() > pitchAutoThreshold)
            pitchServo.setPosition((120 - (tiltMotor.getCurrentPosition() / tiltMotorTicksPerDegree )) / pitchServoTotalDegrees);

        //extend arm

        if (-gamepad2.left_stick_y > 0.75)
            extendArm(extensionMotor.getCurrentPosition() + 100);
        else if (-gamepad2.left_stick_y < -0.75)
            extendArm(extensionMotor.getCurrentPosition() - 100);

        //presets

        if(gamepad2.dpad_up) {;
            tiltTarget = tiltBoard;
            timer.schedule(delayedArm, 500);
        } else if (gamepad2.dpad_down) {
            extendArm(extendBase);
            timer.schedule(delayedTilt, 500);
        }

        //rot arm

        if (-gamepad2.right_stick_y > 0.75)
            tiltTarget += 25;
        else if (-gamepad2.right_stick_y < -0.75)
            tiltTarget -= 25;

        //intake slides

        if (gamepad1.right_trigger > 0.75)
            extendIntake(intakeSlides.getCurrentPosition()+50);
        else if (gamepad1.left_trigger > 0.75)
            extendIntake(intakeSlides.getCurrentPosition()-50);
        else if (gamepad1.b)
            extendIntake(intakeSlidesBase);

        //intake motor

        if (gamepad1.a)
            intakeMotor.setPower(intakePower);
        else
            intakeMotor.setPower(0);

        //pidf loop arm

        armPID(tiltTarget);

    }
    private void extendArm(int targetPos) {
        extensionMotor.setTargetPosition(targetPos);
        extensionMotor.setPower(extendPower);
    }

    private void extendIntake(int targetPos) {
        intakeSlides.setTargetPosition(targetPos);
        intakeSlides.setPower(intakeSlidesPower);
    }
    public void armPID(int target) {

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
}
