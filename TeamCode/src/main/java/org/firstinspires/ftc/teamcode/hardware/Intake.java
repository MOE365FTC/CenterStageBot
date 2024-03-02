package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    CRServo grabLeft, grabRight;
    DcMotor intakeMotor, intakeSlides; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm
    public DigitalChannel grabLeftSwitch, grabRightSwitch;
    Servo transferBeltServo;


    //presets intake
    public static final double intakeSlidesPower = 0.5, intakeMotorPower = 0.7;
    public static final int intakeSlidesBase = 0, intakeSlidesTransfer = 320, intakeSlidesOut = 1440; //tuning needed
    public static ExtendPositions currExtendPos = ExtendPositions.TRANSFER;
    public static RequestedExtendPositions currRequestPos = RequestedExtendPositions.TRANSFER;

    public static final double transferBeltUp = 0.52, transferBeltDown = 0.02;

    double grabServoPower = 0.7;
    boolean oldGrabLeftClicked = true, oldGrabRightClicked = true;
    boolean runGrabL = false, runGrabR = false;


    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    //NEVER RUN THIS IN TELEOP WITHOUT FIRST RUNNING UNFOLD OPMODE OR AUTON
    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        //hardware setup
        grabLeft = hardwareMap.get(CRServo.class, "grabLeft");
        grabRight = hardwareMap.get(CRServo.class, "grabRight");
        grabLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
        grabLeftSwitch = hardwareMap.get(DigitalChannel.class, "grabLeftSwitch");
        grabRightSwitch = hardwareMap.get(DigitalChannel.class, "grabRightSwitch");
        transferBeltServo = hardwareMap.get(Servo.class, "transferBeltServo");

        //transfer belt setup
        if(isAuton) transferBeltServo.setPosition(transferBeltDown);
        else transferBeltServo.setPosition(transferBeltUp);

        //grab setup
        grabLeftSwitch.setMode(DigitalChannel.Mode.INPUT);
        grabRightSwitch.setMode(DigitalChannel.Mode.INPUT);

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
        if(!isAuton) {
            runGrabs();
        }

    }

    public void actuate() {
        //intake slides presets
        if (gamepad1.b && Outtake.currTiltPos != Outtake.TiltPositions.DOWN) { //should only be used right before hang
            extendIntake(intakeSlidesBase);
            currRequestPos = RequestedExtendPositions.BASE;
        } else if(gamepad1.a && Outtake.currTiltPos != Outtake.TiltPositions.DOWN) { //tilt should already be in hover
            Outtake.isAutoTransfer = true;
            extendIntake(intakeSlidesTransfer);
            currRequestPos = RequestedExtendPositions.TRANSFER;
        } else if (gamepad1.x) {
            currRequestPos = RequestedExtendPositions.EXTENDED_FULL; //this will also notify outtake to hover the arm
        }

        if(currRequestPos == RequestedExtendPositions.EXTENDED_FULL && Outtake.currTiltPos == Outtake.TiltPositions.UP) {
            extendIntake(intakeSlidesOut); //only extends to full after arm in hover
            currExtendPos = ExtendPositions.EXTENDED_FULL;
        } else if(currRequestPos == RequestedExtendPositions.TRANSFER && intakeSlides.getCurrentPosition() < intakeSlidesTransfer + 100) {
            currExtendPos = ExtendPositions.TRANSFER; //tells outtake intake slides are ready to transfer
        } else if(currRequestPos == RequestedExtendPositions.BASE && intakeSlides.getCurrentPosition() < 100) {
            currExtendPos = ExtendPositions.BASE;
        }

        //intake motor
        if (gamepad1.right_trigger > 0.3) {
            transferBeltServo.setPosition(transferBeltDown);
            intakeMotor.setPower(intakeMotorPower);
        } else {
            transferBeltServo.setPosition(transferBeltUp);
            intakeMotor.setPower(0);
        }

    }

    private void extendIntake(int targetPos) {
        intakeSlides.setTargetPosition(targetPos);
        intakeSlides.setPower(intakeSlidesPower);
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
        }
    }

    public void runGrabs() { //called once in auton to update runGate variables so that on the next loop of updateGates() the gates will spin once
        runGrabL = true;
        runGrabR = true;
        //could also replace this code by just turning it on and then turning it off with a TimerTask if we cant get limit switches
    }

    public void updateGrabs(){ //needs to be looped in auton
        if(runGrabL) {
            grabLeft.setPower(grabServoPower);
            if(grabLeftSwitch.getState() && !oldGrabLeftClicked) {
                grabLeft.setPower(0);
                runGrabL = false;
            }
            oldGrabLeftClicked = grabLeftSwitch.getState();

        }
        if(runGrabR) {
            grabRight.setPower(grabServoPower);
            if (grabRightSwitch.getState() && !oldGrabRightClicked) {
                grabRight.setPower(0);
                runGrabR = false;
            }
            oldGrabRightClicked = grabRightSwitch.getState();
        }
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
