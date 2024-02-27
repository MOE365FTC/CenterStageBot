package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    CRServo gateLeft, gateRight;
    DcMotor intakeMotor, intakeSlides; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm
    DigitalChannel gateLeftSwitch, gateRightSwitch;
    Servo transferBeltServo;


    //presets intake
    public static final double intakeSlidesPower = 0.7, intakeMotorPower = 0.7; //tuning needed
    public static final int intakeSlidesBase = 0, intakeSlidesTransfer = 30, intakeSlidesOut = 100; //tuning needed
    public static ExtendPositions currExtendPos = ExtendPositions.BASE;

    public static final double transferBeltUp = 0, transferBeltDown = 1.0;


    public static final int MAX_EXTEND_TICKS = 1000; //tune

    double gatePower = 0.7;
    boolean oldGateLeftClicked = true, oldGateRightClicked = true;
    boolean runGateL = false, runGateR = false;


    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        //hardware setup
        gateLeft = hardwareMap.get(CRServo.class, "gateLeft");
        gateRight = hardwareMap.get(CRServo.class, "gateRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
        gateLeftSwitch = hardwareMap.get(DigitalChannel.class, "gateLeftSwitch");
        gateRightSwitch = hardwareMap.get(DigitalChannel.class, "gateRightSwitch");
        transferBeltServo = hardwareMap.get(Servo.class, "transferBelt");

        //transfer belt setup
        transferBeltServo.setPosition(transferBeltUp);

        //gate setup
        gateLeftSwitch.setMode(DigitalChannel.Mode.INPUT);
        gateRightSwitch.setMode(DigitalChannel.Mode.INPUT);

        //extend
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //intake motor
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void actuate() {
        //manual intake slides
        if (gamepad1.b && currExtendPos == ExtendPositions.TRANSFER) {
            extendIntake(intakeSlidesBase);
            currExtendPos = ExtendPositions.BASE;
        } else if(gamepad1.a) {
            extendIntake(intakeSlidesTransfer);
            currExtendPos = ExtendPositions.TRANSFER;
        } else if (gamepad1.x)
            currExtendPos = ExtendPositions.EXTENDED_FULL;
            extendIntake(intakeSlidesOut);


        //intake motor
        if (gamepad1.right_trigger > 0.3) {
            transferBeltServo.setPosition(transferBeltDown);
            intakeMotor.setPower(intakeMotorPower);
        }
        else if(gamepad1.left_trigger > 0.3) {
            transferBeltServo.setPosition(transferBeltDown);
            intakeMotor.setPower(-intakeMotorPower);
        }
        else {
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

    public void runGates() { //called once in auton to update runGate variables so that on the next loop of updateGates() the gates will spin once
        runGateL = true;
        runGateR = true;
        //could also replace this code by just turning it on and then turning it off with a TimerTask if we cant get limit switches
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

    public void telemetryIntake() {
        telemetry.addData("intakeSlides", intakeSlides.getCurrentPosition());
    }

    public enum ExtendPositions {
        EXTENDED_FULL,
        TRANSFER,
        BASE
    }
}
