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
    public static Servo pitchServo; //outtake servos
    DcMotor intakeMotor; //intake motors

    public static DcMotor liftMotor, tiltMotorA, tiltMotorB;//outtake motors //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm

    //presets
    public static boolean isAutoPitch = false;
    public static final double intakeMotorPower = 0.7;
    public static final double grabLeftIn = 0, grabLeftOut = 1, grabRightIn = 1, grabRightOut = 0;
    public static final double intakePitch = 0.0, scorePitch = 0.45, autonPitch = 0.93;
    public static final int tiltStraight = 1984;
    public static final int liftBase = 0; //tuning needed
    public static final int tiltBase = 0;

    public static final double liftPower = 0.8;
    public static final double tiltPower = 0.5;

    public static int liftTarget = liftBase; //can make these static to pass target pos between auto and teleop if needed to fix run to position error, make sure to update this variable in auto functions
    public static int tiltTarget = tiltBase;

    public static final int MIN_TILT_TICKS = 1322; //120deg
    public static final int MAX_TILT_TICKS = 2140;
    public static final int MAX_LIFT_TICKS = 1115;


    //pitch servo parameters
    private final double tiltMotorTicksPerDegree = 1984.0 / 180.0;
    public static double pitchTicksPerDegree = .0042; //1/270

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
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

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
        if (gamepad1.y) {
            runGrabs(true);
        } else {
            runGrabs(false);
        }

        //intake motor
        if (gamepad1.right_trigger > 0.3) {
            intakeMotor.setPower(intakeMotorPower);
        } else if (gamepad1.left_trigger > 0.3) {
            intakeMotor.setPower(-intakeMotorPower);
        } else {
            intakeMotor.setPower(0);
        }

        //manual lift
        if (tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition() > MIN_TILT_TICKS) {
            if (-gamepad2.left_stick_y > 0.75 && liftMotor.getCurrentPosition() <= MAX_LIFT_TICKS - 100)
                liftTarget += 40;
            else if (-gamepad2.left_stick_y < -0.75 && liftMotor.getCurrentPosition() >= 100)
                liftTarget -= 40;
        }
        //manual tilt arm
        if (tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition() > MIN_TILT_TICKS) { //to tilt out use presets and then fine tuning with manual
            if (-gamepad2.right_stick_y > 0.75 && tiltTarget <= MAX_TILT_TICKS - 75) {
                tiltTarget += 25;
            } else if (-gamepad2.right_stick_y < -0.75 && tiltTarget >= MIN_TILT_TICKS + 75)
                tiltTarget -= 25;
        }

        //tilt+lift presets
        if (gamepad2.dpad_up) {
            tiltTarget = tiltStraight;
            isAutoPitch = true;
        } else if (gamepad2.dpad_down) {
            tiltTarget = tiltBase;
            isAutoPitch = false;
        }

        //
        if (tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition() > MIN_TILT_TICKS && isAutoPitch) {
            pitchServo.setPosition(scorePitch - ((((tiltMotorA.getCurrentPosition() + tiltMotorB.getCurrentPosition()) - tiltStraight) / tiltMotorTicksPerDegree) * pitchTicksPerDegree));
        }
    }

    public void autonRunIntake(boolean start) {
        if (start) intakeMotor.setPower(intakeMotorPower);
        else intakeMotor.setPower(0);
    }

    public void autonRunIntake(boolean start, boolean reverse) {
        double mult = reverse ? -1 : 1;
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


    public enum ExtendPositions {
        EXTENDED_FULL,
        BASE
    }

    public enum RequestedExtendPositions {
        EXTENDED_FULL,
        BASE
    }
}
