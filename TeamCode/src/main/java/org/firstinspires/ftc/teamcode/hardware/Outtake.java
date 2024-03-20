package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;
@Deprecated

@Config
public class Outtake {
    private PIDController controller;
    Servo leftIris, rightIris, yawServo;
    public static Servo pitchServo;
    public static DcMotor liftMotor, tiltMotor; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm

    public static double dropPosR = 0.38, dropPosL = 0.62, upPosL = 0.98, upPosR = 0.02;

    //presets
    public static final double leftIrisExpand = 0.59, leftIrisContract = 0.35, rightIrisExpand = 0.62, rightIrisContract = 0.4;
    public static final double intakePitch = 0.0, scorePitch = 0.45, autonPitch = 0.93;
    public static final double yawVertical = 0.32, yawHorizontal = 0.66;
    public static final int tiltTransfer = 0, tiltHover = 450, tiltStraight = 1984;
    public static int liftBase = 0, liftTransfer = 338; //tuning needed

    public static final double liftPower = 0.8;
    public static final double tiltPower = 0.5;

    public static TiltPositions currTiltPos = TiltPositions.READY_TO_INTAKE;

    public int liftTarget = liftTransfer; //can make these static to pass target pos between auto and teleop if needed to fix run to position error, make sure to update this variable in auto functions


    public static final int MIN_TILT_TICKS = 1322; //120deg
    public static final int MAX_TILT_TICKS = 2140;
    public static final int MAX_LIFT_TICKS = 1115;


    public static int tiltTarget = tiltTransfer; //initialize to tiltBase

    //pitch servo parameters
    private final double tiltMotorTicksPerDegree= 1984.0/180.0;
    public static double pitchTicksPerDegree = .0042; //1/270
    boolean isAutoPitch = false;

    public static boolean isAutoTransfer = true;

    //intake state
    private Intake.ExtendPositions oldExtendState;

    //timers
    TimerTask delayedExtendToLow, delayedTiltToTransfer;
    boolean tiltScheduled = false;
    boolean tiltScheduledForPreset = false;
    Timer timer;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    public Outtake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        //hardware setup
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        yawServo = hardwareMap.get(Servo.class, "yawServo");

        //iris setup
        if(isAuton) {
            leftIris.setPosition(leftIrisExpand);
            rightIris.setPosition(rightIrisExpand);
        } else {
            leftIris.setPosition(leftIrisContract);
            rightIris.setPosition(rightIrisContract);
        }


        //wrist setup
        if(isAuton) {
            pitchServo.setPosition(autonPitch);
            yawServo.setPosition(yawHorizontal);
        } else {
            pitchServo.setPosition(intakePitch);
            yawServo.setPosition(yawVertical);
        }

        //tilt setup
        tiltMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(isAuton) {
            tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tiltMotor.setTargetPosition(0);
        }
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(isAuton) { //don't want to zero in teleop since its already extended to transfer
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TimerTask setup
        timer = new Timer("timer");
        delayedTiltToTransfer = new TimerTask() {
            @Override
            public void run() {
                tiltTarget = tiltTransfer;
            }
        };
    }

    public void actuate() {
        //iris
        if(currTiltPos == TiltPositions.READY_TO_INTAKE) {
            leftIris.setPosition(leftIrisContract);
            rightIris.setPosition(rightIrisContract);
        } else {
            if(gamepad2.left_bumper)
                leftIris.setPosition(leftIrisContract);
            else
                leftIris.setPosition(leftIrisExpand);

            if(gamepad2.right_bumper)
                rightIris.setPosition(rightIrisContract);
            else
                rightIris.setPosition(rightIrisExpand);
        }

        //pitch servo
        if (tiltMotor.getCurrentPosition() > MIN_TILT_TICKS && isAutoPitch)
            pitchServo.setPosition(scorePitch - (((tiltMotor.getCurrentPosition()-tiltStraight)/tiltMotorTicksPerDegree) * pitchTicksPerDegree));

        //manual lift
        if(tiltMotor.getCurrentPosition() > MIN_TILT_TICKS) {
            if (-gamepad2.left_stick_y > 0.75 && liftMotor.getCurrentPosition() <= MAX_LIFT_TICKS - 100)
                liftTarget += 40;
            else if (-gamepad2.left_stick_y < -0.75 && liftMotor.getCurrentPosition() >= 100)
                liftTarget -= 40;
        }

        //manual tilt arm
        if(tiltMotor.getCurrentPosition() > MIN_TILT_TICKS) { //to tilt out use presets and then fine tuning with manual
            if (-gamepad2.right_stick_y > 0.75 && tiltTarget <= MAX_TILT_TICKS - 75) {
                tiltTarget += 25;
            } else if (-gamepad2.right_stick_y < -0.75 && tiltTarget >= MIN_TILT_TICKS + 75)
                tiltTarget -= 25;
        }

        //tilt+lift presets
        if(gamepad2.dpad_up) {
            currTiltPos = TiltPositions.READY_TO_OUTTAKE; //tells irises to expand
            isAutoTransfer = false;
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    tiltTarget = tiltStraight;
                    isAutoPitch = true;
                }
            }, 200);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    yawServo.setPosition(yawHorizontal);
                }
            }, 900);
            tiltScheduled = false;
        } else if (gamepad2.dpad_down && Intake.currExtendPos == Intake.ExtendPositions.TRANSFER) {
            currTiltPos = TiltPositions.READY_TO_INTAKE;
            pitchServo.setPosition(intakePitch);
            isAutoPitch = false;
            isAutoTransfer = true;
//            liftTarget = liftTransfer;
//            yawServo.setPosition(yawVertical);
//            if(!tiltScheduledForPreset) {
//                timer.schedule(new TimerTask() {
//                    @Override
//                    public void run() {
//                        tiltTarget = tiltTransfer;
//                    }
//                }, 400);
//                tiltScheduledForPreset = true;
//            }
//            timer.schedule(new TimerTask() {
//                @Override
//                public void run() {
//                    currTiltPos = TiltPositions.READY_TO_OUTTAKE;
//                }
//            }, 2400);
        }

        if(tiltMotor.getCurrentPosition() < tiltHover - 100) {
            tiltScheduledForPreset = false;
        }

        if(isAutoTransfer) {
            if(Intake.currRequestPos == Intake.RequestedExtendPositions.EXTENDED_FULL) { //intake wants to extend
                currTiltPos = TiltPositions.READY_TO_INTAKE;
                hoverArm();
                tiltScheduled = false;
            } else if(Intake.currExtendPos == Intake.ExtendPositions.TRANSFER) { //intake is ready to transfer
                pitchServo.setPosition(intakePitch);
                yawServo.setPosition(yawVertical);
                isAutoPitch = false;
                liftTarget = liftTransfer;
                if(!tiltScheduled) {
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            tiltTarget = tiltTransfer;
                        }
                    }, 400);
                    tiltScheduled = true;
                }
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        currTiltPos = TiltPositions.READY_TO_OUTTAKE;
                    }
                }, 2400);
            }
        }

        if(Intake.currExtendPos == Intake.ExtendPositions.BASE) { //arm is already in hover at this point
            pitchServo.setPosition(autonPitch);
            yawServo.setPosition(yawHorizontal);
            liftTarget = liftBase;
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    tiltTarget = tiltTransfer;
                }
            }, 2400);
        }

        liftArm(liftTarget);
        tiltArm(tiltTarget);
    }


    public void hoverArm() {
        liftTarget = liftTransfer;
        tiltTarget = tiltHover;
    }

    public void setPitchServo(double pos) {
        pitchServo.setPosition(pos);
    }

    public void setYawServo(double pos) {
        yawServo.setPosition(pos);
    }

    private void tiltArm(int targetPos) {
        tiltMotor.setTargetPosition(targetPos);
        tiltMotor.setPower(tiltPower);
    }

    private void liftArm(int targetPos) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(liftPower);
    }


    public void autonIris(boolean expand) { //auton method for iris control
        leftIris.setPosition(expand ? leftIrisExpand : leftIrisContract);
        rightIris.setPosition(expand ? rightIrisExpand : rightIrisContract);
    }

    public void autonLeftIris(boolean expand) { //individual auton iris control
        leftIris.setPosition(expand ? leftIrisExpand : leftIrisContract);
    }

    public void autonRightIris(boolean open) { //individual auton iris control
        rightIris.setPosition(open ? rightIrisExpand : rightIrisContract);
    }

    public void autonTilt(autonTiltPositions pos) { //use in teleop and auto by changing the tiltTarget variable (this will be automatically called at the end of loops)
        switch(pos) {
            case BASE:
                tiltArm(tiltTransfer);
                break;
            case HOVER:
                tiltArm(tiltHover);
                break;
            case SCORE:
                tiltArm(tiltStraight + 150);
        }
    }

    public void autonLift(autonLiftPositions pos) {
        switch(pos) {
            case BASE:
                liftArm(liftBase);
                break;
            case TRANSFER:
                liftArm(liftTransfer);
                break;
            case EXTEND:
                liftArm(MAX_LIFT_TICKS - 100);
            default:
                liftArm(liftBase);
        }
    }

    public void telemetryOuttake() {
        String liftString = "T: " + liftTarget + " | A: " + liftMotor.getCurrentPosition();
        String tiltString = "T: " + tiltTarget + " | A: " + tiltMotor.getCurrentPosition();
        telemetry.addData("lift T", liftTarget);
        telemetry.addData("lift A", liftMotor.getCurrentPosition());
        telemetry.addData("tilt T", tiltTarget);
        telemetry.addData("tilt A", tiltMotor.getCurrentPosition());
    }

    public void zero() {
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setTargetPosition(0);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public enum autonLiftPositions {
        BASE,
        TRANSFER,
        EXTEND,
    }

    public enum autonTiltPositions {
        BASE,
        HOVER,
        SCORE
    }

    public enum TiltPositions {
        READY_TO_INTAKE,
        READY_TO_OUTTAKE
    }

}
