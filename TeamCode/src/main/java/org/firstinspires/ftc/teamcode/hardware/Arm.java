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
public class Arm {
    private PIDController controller; //pidf
    public static double p = 0.0024, i = 0.135, d = 0.0001, f = 0.2;
    Servo grabLeft, grabRight; //intake servos
    Servo pitchServo, boxGateMain, boxGateAux; //outtake servos
    DcMotor intakeMotor; //intake motors //control 2

    DcMotorEx tiltMotorA; //expansion 1 and 2, control 2 for extend
    DcMotor extensionMotor, tiltMotorB;//outtake motors //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm

    //presets
    public static boolean isAutoPitch = false;
    public static final double intakeMotorPower = 1.0;
    public static final double grabLeftIn = 0.0, grabLeftOut = 1.0, grabRightIn = 1.0, grabRightOut = 0.2;
    public static final double basePitch = 0.02, intakePitch = 0.14, scorePitch = 0.84;
    public static final int tiltStraight = 1987, tiltHang = 900, tiltBase = 0;
    public static final int extendBase = 0; //tuning needed

    public static final double extendPower = 0.8;
    public static final double tiltPower = 0.5;

    public static boolean isHang = false;
    public static double hangTiltPower = 0.2;

    public static int extendTarget = extendBase; //can make these static to pass target pos between auto and teleop if needed to fix run to position error, make sure to update this variable in auto functions
    public static int tiltTarget = tiltBase;

    public static final int MIN_TILT_TICKS = 850; //temp
    public static final int MAX_TILT_TICKS = 2140;
    public static final int MAX_EXTEND_TICKS = 1115;


    public static final double pitchTicksPerDegree = 1.0 / 180.0; //1/270 <-- check range of motion (servo angle/255 * 355 --> real operating range in degrees)
    //pitch servo parameters
    private static final double tiltMotorTicksPerDegree = 1987.0 / 180.0;

    public static final double boxOpenMain = 0.11;//needs tuning
    public static final double boxCloseMain = 0.03;//needs tuning

    public static final double boxOpenAux = 0.9;//needs tuning
    public static final double boxCloseAux = 1.0;//needs tuning

    //intake state
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    Timer timer;

    public Arm(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        timer = new Timer();

        //pidf setup
        controller = new PIDController(p, i ,d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //hardware setup

        extensionMotor = hardwareMap.get(DcMotor.class, "extendMotor");
        tiltMotorA = hardwareMap.get(DcMotorEx.class, "tiltMotorA");
        tiltMotorB = hardwareMap.get(DcMotor.class, "tiltMotorB");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        boxGateMain = hardwareMap.get(Servo.class, "boxGateMain");
        boxGateAux = hardwareMap.get(Servo.class, "boxGateAux");
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");

        //tilt setup
//        tiltMotorA.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltMotorB.setDirection(DcMotorSimple.Direction.REVERSE);

        if(isAuton) {
            tiltMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tiltMotorA.setTargetPosition(0);
        }
//        tiltMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        tiltMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        tiltMotorB.setTargetPosition(0);
//        tiltMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //extension
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(isAuton) {
            extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extensionMotor.setTargetPosition(0);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //intake motor
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //grab servos
        if (isAuton) {
            grabRight.setPosition(grabRightIn);
            grabLeft.setPosition(grabLeftIn);
        } else {
            grabRight.setPosition(grabRightOut);
            grabLeft.setPosition(grabLeftOut);
        }

        //box gate
        pitchServo.setPosition(intakePitch);
        boxGateMain.setPosition(boxOpenMain);
        boxGateAux.setPosition(boxCloseAux);

        isHang = false;
        isAutoPitch = false;

    }

    public void actuate() {
        //grabbers
        runGrabs(gamepad1.y);

        //intake motor
        if (0.3 < gamepad1.right_trigger) {
            intakeMotor.setPower(intakeMotorPower);
//            boxGateMain.setPosition(boxOpenMain);
//            if(tiltMotorA.getCurrentPosition() < tiltBase + 200) pitchServo.setPosition(intakePitch);
        } else if (0.3 < gamepad1.left_trigger) {
            intakeMotor.setPower(-intakeMotorPower);
//            boxGateMain.setPosition(boxOpenMain);
//            if(tiltMotorA.getCurrentPosition() < tiltBase + 200) pitchServo.setPosition(intakePitch);
        } else {
            if(!isAutoPitch) {
//                boxGateMain.setPosition(boxCloseMain);
            }
            intakeMotor.setPower(0);
        }

        //manual lift
        if (tiltMotorA.getCurrentPosition() >= MIN_TILT_TICKS) {
            if (-gamepad2.left_stick_y >= 0.75 && extensionMotor.getCurrentPosition() < MAX_EXTEND_TICKS-40)
                extendTarget += 40;
            else if (-gamepad2.left_stick_y <= -0.75 && extensionMotor.getCurrentPosition() >= 40)
                extendTarget -= 40;
        }
        //manual tilt arm
        if (tiltMotorA.getCurrentPosition() >= MIN_TILT_TICKS) { //to tilt out use presets and then fine tuning with manual
            if (-gamepad2.right_stick_y >= 0.75 && tiltMotorA.getCurrentPosition() < MAX_TILT_TICKS - 25) {
                tiltTarget += 25;
            } else if (-gamepad2.right_stick_y <= -0.75 && tiltMotorA.getCurrentPosition() >= 25)
                tiltTarget -= 25;
        }

        //presets
        if (gamepad2.dpad_up) {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    tiltTarget = tiltStraight;
                }
            }, 75);
            isAutoPitch = true;
            pitchServo.setPosition(basePitch);
            boxGateMain.setPosition(boxCloseMain); //???
        } else if (gamepad2.dpad_down) {
            extendTarget = extendBase;
            isAutoPitch = false;
            pitchServo.setPosition(intakePitch);
            boxGateMain.setPosition(boxOpenMain); //???
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    tiltTarget = tiltBase;
                }
            }, 500);
        } else if (gamepad2.y) {
            extendTarget = extendBase;
            isAutoPitch = false;
            tiltTarget = tiltHang;
        } else if (gamepad2.b) {
            isHang = true;
        } else if (gamepad2.x){
            isHang = false;
        }

        //auto pitch servo
        if (tiltMotorA.getCurrentPosition() > MIN_TILT_TICKS + 350 && isAutoPitch) {
            double servoCalcPos = (scorePitch - ((((tiltMotorA.getCurrentPosition()) - tiltStraight) / tiltMotorTicksPerDegree) * pitchTicksPerDegree));
            pitchServo.setPosition(Math.min(servoCalcPos, 0.93));
        }

        //box gate
        if(isAutoPitch) { //prevents conflicting commands when retracting (box set to open but this code would set it to closed before arm is past min tilt)
            if (gamepad2.right_bumper && tiltMotorA.getCurrentPosition() >= MIN_TILT_TICKS) {
                boxGateMain.setPosition(boxOpenMain);
            } else if (tiltMotorA.getCurrentPosition() >= MIN_TILT_TICKS) {
                boxGateMain.setPosition(boxCloseMain);
            }
        }

        extendArm(extendTarget);
        if(!isHang) {
            tiltArm(tiltTarget);
        } else {
            tiltMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            tiltMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            tiltMotorA.setPower(gamepad2.right_trigger > 0.3 ? -1: -hangTiltPower);
            tiltMotorB.setPower(gamepad2.right_trigger > 0.3 ? -1: -hangTiltPower);
        }
    }

    public void tiltArm(int targetPos) { //loop this function in auton
        controller.setPID(p, i, d);
        int armPos = tiltMotorA.getCurrentPosition();
        double pid = controller.calculate(armPos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos/tiltMotorTicksPerDegree)) * f;

        double power = pid + ff;
        telemetry.addData("ff", ff);
        telemetry.addData("power", power);
        tiltMotorA.setPower(power);
        tiltMotorB.setPower(power);
    }

    private void extendArm(int targetPos) {
        extensionMotor.setTargetPosition(targetPos);
        extensionMotor.setPower(extendPower);
    }


    public void autonExtend(autonExtendPositions pos) {
        switch (pos) {
            case BASE:
                extendArm(extendBase);
                break;
            case EXTEND:
                extendArm(MAX_EXTEND_TICKS - 100);
            default:
                extendArm(extendBase);
        }
    }

    public void autonRunIntake(boolean start) {
        if (start) intakeMotor.setPower(intakeMotorPower);
        else intakeMotor.setPower(0);
    }

    public void autonRunIntake(boolean start, boolean reverse) {
        final double mult = reverse ? -1 : 1;
        if (start) intakeMotor.setPower(mult * intakeMotorPower);
        else intakeMotor.setPower(0);
    }

    public void runGrabs(boolean in) {
        if (in) {
            grabLeft.setPosition(grabLeftIn);
            grabRight.setPosition(grabRightIn);
        } else {
            grabLeft.setPosition(grabLeftOut);
            grabRight.setPosition(grabRightOut);
        }
    }

    public void autonSetPitchServo(double pos) {
        pitchServo.setPosition(pos);
    }

    public void autonSetBoxGate(boolean open) {
        if(open) boxGateMain.setPosition(boxOpenMain);
        else boxGateMain.setPosition(boxCloseMain);
    }

    public void resetEncoders() {
        tiltMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotorA.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void telemetryArm() {
        telemetry.addData("ext T", extendTarget);
        telemetry.addData("ext A", extensionMotor.getCurrentPosition());
        telemetry.addData("tilt T", tiltTarget);
        telemetry.addData("tilt A", tiltMotorA.getCurrentPosition());
        telemetry.update();
    }

    public enum autonExtendPositions {
        BASE,
        EXTEND,
    }

    public enum autonTiltPositions {
        BASE,
        SCORE
    }

}
