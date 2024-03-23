package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Control {
    private PIDController controller; //pidf
    public static Servo grabLeft, grabRight; //intake servos
    public static Servo pitchServo, boxGate; //outtake servos
    public static DcMotor intakeMotor; //intake motors

    public static DcMotor liftMotor, tiltMotorA, tiltMotorB;//outtake motors //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm

    //presets
    public static boolean isAutoPitch = false;
    public static final double intakeMotorPower = 0.7;
    public static final double grabLeftIn = 0, grabLeftOut = 1, grabRightIn = 1, grabRightOut = 0;
    public static final double intakePitch = 0.0, scorePitch = 0.45, autonPitch = 0.93;
    public static final int tiltStraight = 1984;
    public static final int liftBase = 0; //tuning needed
    public static final int tiltBase = 0; //tuning needed

    public static final double liftPower = 0.8;
    public static final double tiltPower = 0.5;

    public static int liftTarget = liftBase; //can make these static to pass target pos between auto and teleop if needed to fix run to position error, make sure to update this variable in auto functions
    public static int tiltTarget = tiltBase;

    public static final int MIN_TILT_TICKS = 1322; //120deg
    public static final int MAX_TILT_TICKS = 2140;
    public static final int MAX_LIFT_TICKS = 1115;


    public static final double pitchTicksPerDegree = 0.0042; //1/270
    //pitch servo parameters
    private static final double tiltMotorTicksPerDegree = 1984.0 / 180.0;

    public static final double boxOpen = 1;//needs tuning
    public static final double boxClose = 0;//needs tuning

    //intake state
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    public Control(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        //hardware setup

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        tiltMotorA = hardwareMap.get(DcMotor.class, "tiltMotorA");
        tiltMotorB = hardwareMap.get(DcMotor.class, "tiltMotorB");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        boxGate = hardwareMap.get(Servo.class, "boxGate");
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");

        //tilt setup
        tiltMotorA.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuton) {
            tiltMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tiltMotorA.setTargetPosition(0);
        }
        tiltMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuton) {
            tiltMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tiltMotorB.setTargetPosition(0);
        }
        tiltMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuton) { //don't want to zero in teleop since its already extended to transfer
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //intake motor
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //grab servos
        if (isAuton) {
            grabRight.setPosition(grabRightIn);
            grabLeft.setPosition(grabLeftIn);
        } else {
            grabRight.setPosition(grabRightOut);
            grabLeft.setPosition(grabLeftOut);
        }
    }

    public void actuate() {
        //grabbers
        runGrabs(gamepad1.y);

        //intake motor
        if (0.3 < gamepad1.right_trigger) {
            intakeMotor.setPower(intakeMotorPower);
        } else if (0.3 < gamepad1.left_trigger) {
            intakeMotor.setPower(-intakeMotorPower);
        } else {
            intakeMotor.setPower(0);
        }

        //manual lift
        if (MIN_TILT_TICKS < tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition()) {
            if (0.75 < -gamepad2.left_stick_y && MAX_LIFT_TICKS - 100 >= liftMotor.getCurrentPosition())
                liftTarget += 40;
            else if (-0.75 > -gamepad2.left_stick_y && 100 <= liftMotor.getCurrentPosition())
                liftTarget -= 40;
        }
        //manual tilt arm
        if (MIN_TILT_TICKS < tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition()) { //to tilt out use presets and then fine tuning with manual
            if (0.75 < -gamepad2.right_stick_y && MAX_TILT_TICKS - 75 >= tiltTarget) {
                tiltTarget += 25;
            } else if (-0.75 > -gamepad2.right_stick_y && MIN_TILT_TICKS + 75 <= tiltTarget)
                tiltTarget -= 25;
        }

        //tilt+lift presets
        if (gamepad2.dpad_up) {
            tiltTarget = tiltStraight;
            isAutoPitch = true;
            setBoxGate(boxClose);
        } else if (gamepad2.dpad_down) {
            tiltTarget = tiltBase;
            isAutoPitch = false;
            setBoxGate(boxClose);
        }

        //auto pitch servo
        if (MIN_TILT_TICKS < tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition() && isAutoPitch) {
            pitchServo.setPosition(scorePitch - ((((tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition()) - tiltStraight) / tiltMotorTicksPerDegree) * pitchTicksPerDegree));
        }

        //box gate
        if (gamepad2.right_bumper && MIN_TILT_TICKS < tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition()) {
            setBoxGate(boxOpen);
        } else if (gamepad2.left_bumper) {
            setBoxGate(boxClose);
        }

        liftArm(liftTarget);
        tiltArm(tiltTarget);
    }

    private void tiltArm(int targetPos) {
        tiltMotorA.setTargetPosition(targetPos);
        tiltMotorB.setTargetPosition(targetPos);
        tiltMotorA.setPower(tiltPower);
        tiltMotorB.setPower(tiltPower);
    }

    private void liftArm(int targetPos) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(liftPower);
    }

    public void autonTilt(autonTiltPositions pos) { //use in teleop and auto by changing the tiltTarget variable (this will be automatically called at the end of loops)
        switch (pos) {
            case BASE:
                tiltArm(tiltBase);
                break;
            case SCORE:
                tiltArm(tiltStraight + 150);
        }
    }

    public void autonLift(autonLiftPositions pos) {
        switch (pos) {
            case BASE:
                liftArm(liftBase);
                break;
            case EXTEND:
                liftArm(MAX_LIFT_TICKS - 100);
            default:
                liftArm(liftBase);
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

    public void setPitchServo(double pos) {
        pitchServo.setPosition(pos);
    }

    public void setBoxGate(double pos) {
        boxGate.setPosition(pos);
    }

    public enum ExtendPositions {
        EXTENDED_FULL,
        BASE
    }

    public enum RequestedExtendPositions {
        EXTENDED_FULL,
        BASE
    }

    public void telemetryOuttake() {
//        String liftString = "T: " + liftTarget + " | A: " + liftMotor.getCurrentPosition();
//        String tiltString = "T: " + tiltTarget + " | A: " + tiltMotor.getCurrentPosition();
        telemetry.addData("lift T", liftTarget);
        telemetry.addData("lift A", liftMotor.getCurrentPosition());
        telemetry.addData("tilt T", tiltTarget);
        telemetry.addData("tilt A", tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition());
    }

    public enum autonLiftPositions {
        BASE,
        EXTEND,
    }

    public enum autonTiltPositions {
        BASE,
        SCORE
    }

    public enum TiltPositions {
        READY_TO_INTAKE,
        READY_TO_OUTTAKE
    }

}
