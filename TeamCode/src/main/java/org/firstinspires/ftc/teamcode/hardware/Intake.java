package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;
@Deprecated

@Config
public class Intake {
    Servo grabLeft, grabRight;
    DcMotor intakeMotor, intakeSlides; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm
    public DigitalChannel grabLeftSwitch, grabRightSwitch;
    Servo transferBeltServo;
    Servo PTO;

    //PTO
    boolean readyToHang = false;
    double lockedPos = 0.95, releasedPos = 0.45;
    double hangPower = 1.0;


    //presets intake
    public static final double intakeSlidesPower = 1, intakeMotorPower = 0.7;
    public static final int intakeSlidesBase = 0, intakeSlidesTransfer = 330, intakeSlidesOut = 1440; //tuning needed
    public static ExtendPositions currExtendPos = ExtendPositions.TRANSFER;
    public static RequestedExtendPositions currRequestPos = RequestedExtendPositions.TRANSFER;

    public static final double transferBeltUp = 0.52, transferBeltDown = 0.02;

    public static double grabLeftIn = 0, grabLeftOut = 1, grabRightIn = 1, grabRightOut = 0;


    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    Timer timer;

    //NEVER RUN THIS IN TELEOP WITHOUT FIRST RUNNING UNFOLD OPMODE OR AUTON
    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        //hardware setup
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
        transferBeltServo = hardwareMap.get(Servo.class, "transferBeltServo");
        PTO = hardwareMap.get(Servo.class, "PTO");

        PTO.setPosition(lockedPos);

        //transfer belt setup
        if(isAuton) transferBeltServo.setPosition(transferBeltDown);
        else transferBeltServo.setPosition(transferBeltUp);

        //extend
        if(isAuton) {
            intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlides.setTargetPosition(0);
            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else { //don't reset encoders when in teleop since the intake is already at transfer pos
            intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //intake motor
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeSlides.setPower(intakeSlidesPower);

        //grab servos
        if(isAuton) {
            grabRight.setPosition(grabRightIn);
            grabLeft.setPosition(grabLeftIn);
        } else {
            grabRight.setPosition(grabRightOut);
            grabLeft.setPosition(grabLeftOut);
        }

        timer = new Timer("intakeTimer");

    }

    public void actuate() {
        if(gamepad1.y){
            runGrabs(true);
        } else {
            runGrabs(false);
        }

        actuatePTO();

        //intake slides presets
        if (gamepad1.b && Outtake.tiltMotor.getCurrentPosition() > Outtake.tiltHover - 30) { //should only be used right before hang
            Outtake.pitchServo.setPosition(0.93);
            transferBeltServo.setPosition(transferBeltDown);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    extendIntake(intakeSlidesBase);
                    currRequestPos = RequestedExtendPositions.BASE;
                }
            }, 180);
        } else if(gamepad1.a && Outtake.tiltMotor.getCurrentPosition() > Outtake.tiltHover - 30) { //tilt should already be in hover
            Outtake.isAutoTransfer = true;
            extendIntake(intakeSlidesTransfer);
            currRequestPos = RequestedExtendPositions.TRANSFER;
        } else if (gamepad1.x) {
            currRequestPos = RequestedExtendPositions.EXTENDED_FULL; //this will also notify outtake to hover the arm
        }

        if(currRequestPos == RequestedExtendPositions.EXTENDED_FULL && Outtake.tiltMotor.getCurrentPosition() > Outtake.tiltHover - 30) {
            extendIntake(intakeSlidesOut); //only extends to full after arm in hover
            currExtendPos = ExtendPositions.EXTENDED_FULL;
        } else if(currRequestPos == RequestedExtendPositions.TRANSFER && intakeSlides.getCurrentPosition() < intakeSlidesTransfer + 100) {
            currExtendPos = ExtendPositions.TRANSFER; //tells outtake intake slides are ready to transfer
        } else if(currRequestPos == RequestedExtendPositions.BASE && intakeSlides.getCurrentPosition() < 100) {
            currExtendPos = ExtendPositions.BASE;
        }

        //intake motor
        if(!readyToHang) {
            if (gamepad1.right_trigger > 0.3) {
                transferBeltServo.setPosition(transferBeltDown);
                intakeMotor.setPower(intakeMotorPower);
            } else if (gamepad1.left_trigger > 0.3) {
                transferBeltServo.setPosition(transferBeltDown);
                intakeMotor.setPower(-intakeMotorPower);
            } else {
                transferBeltServo.setPosition(transferBeltUp);
                intakeMotor.setPower(0);
            }
        }

    }

    public void actuatePTO() {
        if(readyToHang) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.right_trigger > 0.3) {
                intakeMotor.setPower(hangPower);
            } else if (gamepad2.left_trigger > 0.3) {
                intakeMotor.setPower(-hangPower);
            } else {
                intakeMotor.setPower(0);
            }
        }

        if(gamepad2.b && gamepad2.x) {
            readyToHang = true;
            PTO.setPosition(releasedPos);
        }
    }

    private void extendIntake(int targetPos) {
        if(!readyToHang) {
            intakeSlides.setTargetPosition(targetPos);
            intakeSlides.setPower(intakeSlidesPower);
        }
    }


    public void autonIntakeSlides(ExtendPositions pos) {
        switch(pos) {
            case BASE:
                extendIntake(intakeSlidesBase);
                break;
            case TRANSFER:
                extendIntake(intakeSlidesTransfer);
                break;
            case EXTENDED_FULL:
                extendIntake(intakeSlidesOut);
                break;
        }
    }

    public void autonRunIntake(boolean start) {
        if(start) intakeMotor.setPower(intakeMotorPower);
        else intakeMotor.setPower(0);
    }

    public void autonRunIntake(boolean start, boolean reverse) {
        double mult = reverse ? -1 : 1;
        if(start) intakeMotor.setPower(mult * intakeMotorPower);
        else intakeMotor.setPower(0);
    }

//    public void runGrabs() { //called once in auton to update runGate variables so that on the next loop of updateGates() the gates will spin once
//        runGrabL = true;
//        runGrabR = true;
//        //could also replace this code by just turning it on and then turning it off with a TimerTask if we cant get limit switches
//    }

    public void runGrabs(boolean in) {
        if(in) {
            grabLeft.setPosition(grabLeftIn);
            grabRight.setPosition(grabRightIn);
        } else {
            grabLeft.setPosition(grabLeftOut);
            grabRight.setPosition(grabRightOut);
        }
    }

    public void setTransferBeltServo(boolean up){
        if(up) transferBeltServo.setPosition(transferBeltUp);
        else transferBeltServo.setPosition(transferBeltDown);
    }

    public void updateGrabs(){ //needs to be looped in auton
//        if(runGrabL) {
//            grabLeft.setPower(grabServoPower);
//            if(grabLeftSwitch.getState() && !oldGrabLeftClicked) {
//                grabLeft.setPower(0);
//                runGrabL = false;
//            }
//            oldGrabLeftClicked = grabLeftSwitch.getState();
//
//        }
//        if(runGrabR) {
//            grabRight.setPower(grabServoPower);
//            if (grabRightSwitch.getState() && !oldGrabRightClicked) {
//                grabRight.setPower(0);
//                runGrabR = false;
//            }
//            oldGrabRightClicked = grabRightSwitch.getState();
//        }
    }

    public void zero() {
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void telemetryIntake() {
        telemetry.addData("intakeSlides", intakeSlides.getCurrentPosition());
    }

    public enum ExtendPositions {
        EXTENDED_FULL,
        TRANSFER,
        BASE
    }

    public enum RequestedExtendPositions {
        EXTENDED_FULL,
        TRANSFER,
        BASE
    }
}
