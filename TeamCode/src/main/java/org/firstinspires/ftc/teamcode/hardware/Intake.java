package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;

    DcMotor intakeMotor;

    Servo leftIntake, rightIntake;

    DigitalChannel limitLeft, limitRight;
    boolean turnOnIntake = true;
    boolean leftIntakeStopped, rightIntakeStopped = false;
    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        intakeMotor = hardwareMap.get(DcMotor.class, "IM");

        leftIntake = hardwareMap.get(Servo.class, "LIS");
        rightIntake = hardwareMap.get(Servo.class, "RIS");

        limitLeft = hardwareMap.get(DigitalChannel.class, "LL");
        limitRight = hardwareMap.get(DigitalChannel.class, "LR");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake.setPosition(45);
        rightIntake.setPosition(45);
        intakeMotor.setPower(0);
    }


    public void actuate(){
        if(turnOnIntake) {
            if (gamepad1.left_trigger > 0) {
                intakeMotor.setPower(1);
            } else if (gamepad1.right_trigger > 0) {
                intakeMotor.setPower(-1);
            }
        }

        if (limitRight.getState()) {
            rightIntakeStopped = true;
            rightIntake.setPosition(45);
            gamepad1.rumble(100);
        }

        if (limitLeft.getState()) {
            leftIntakeStopped = true;
            leftIntake.setPosition(45);
            gamepad1.rumble(100);
        }

        if(!rightIntakeStopped)
            rightIntake.setPosition(180);

        if(!leftIntakeStopped)
            leftIntake.setPosition(180);

        if(leftIntakeStopped && rightIntakeStopped)
            turnOnIntake = false;

        /*
        *
        * CHICKEN SCRAP
        *
        * if(dispenser.hasPlaced()){
        *   turnOnIntake = true;
        *   leftIntakeStopped = false;
        *   rightIntakeStopped = false;
        * }
        * */
    }

    public void autonActuate(boolean startIntake){
        if(startIntake){
            intakeMotor.setPower(1);
            if (limitRight.getState())
                rightIntake.setPosition(45);
            else
                rightIntake.setPosition(180);

            if (limitLeft.getState())
                leftIntake.setPosition(45);
            else
                leftIntake.setPosition(180);
        }else{
            intakeMotor.setPower(0);
            rightIntake.setPosition(45);
            leftIntake.setPosition(45);
        }
    }
}
