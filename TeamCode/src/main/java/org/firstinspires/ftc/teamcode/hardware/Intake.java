package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;
    Dispenser dispenser;

    DcMotor intakeMotor;

    Servo leftIntake, rightIntake;

    DigitalChannel limitLeft, limitRight;
    double intakeSpeed = 1.0;
    double intakeUp = 0.5, intakeDown = 0.0;
    boolean leftIntakeStopped, rightIntakeStopped = false;
    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Dispenser dispenser){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.dispenser = dispenser;

        intakeMotor = hardwareMap.get(DcMotor.class, "IM");

        leftIntake = hardwareMap.get(Servo.class, "LIS");
        rightIntake = hardwareMap.get(Servo.class, "RIS");

        limitLeft = hardwareMap.get(DigitalChannel.class, "LL");
        limitRight = hardwareMap.get(DigitalChannel.class, "LR");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake.setPosition(intakeDown);
        rightIntake.setPosition(intakeDown);
    }


    public void actuate(){
        if (gamepad1.left_trigger > 0) {
            intakeMotor.setPower(intakeSpeed);
        } else if (gamepad1.right_trigger > 0) {
            intakeMotor.setPower(-intakeSpeed);
        } else {
            intakeMotor.setPower(0.0);
        }

        if (limitRight.getState()) {
            rightIntakeStopped = true;
            rightIntake.setPosition(intakeUp);
            gamepad1.rumble(100);
        }

        if (limitLeft.getState()) {
            leftIntakeStopped = true;
            leftIntake.setPosition(intakeUp);
            gamepad1.rumble(100);
        }

        if(!rightIntakeStopped)
            rightIntake.setPosition(intakeDown);

        if(!leftIntakeStopped)
            leftIntake.setPosition(intakeDown);


        if(dispenser.leftIrisIsEmpty()) leftIntakeStopped = false;
        if(dispenser.rightIrisIsEmpty()) rightIntakeStopped = false;
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