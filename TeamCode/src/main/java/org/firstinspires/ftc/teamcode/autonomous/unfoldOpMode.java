package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Deprecated
@Autonomous(group = "Old")
@Config
public class unfoldOpMode extends LinearOpMode {

    DcMotor intakeSlides, liftMotor, tiltMotor;
    Servo transferBeltServo, pitchServo, yawServo, PTO, irisLeft, irisRight;
    public static int transferTicks = 338;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        transferBeltServo = hardwareMap.get(Servo.class, "transferBeltServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        yawServo = hardwareMap.get(Servo.class, "yawServo");
        PTO = hardwareMap.get(Servo.class, "PTO");
        irisLeft = hardwareMap.get(Servo.class, "leftIris");
        irisRight = hardwareMap.get(Servo.class, "rightIris");
        irisLeft.setPosition(0.35);
        irisRight.setPosition(0.4);
        yawServo.setPosition(0.66); //horizontal
        pitchServo.setPosition(0.93);

        tiltMotor.setDirection(DcMotorSimple.Direction.REVERSE); //REVERSE TILT


        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setTargetPosition(0);
        liftMotor.setTargetPosition(0);
        tiltMotor.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PTO.setPosition(0.95);

        waitForStart();

        tiltMotor.setTargetPosition(450);
        liftMotor.setTargetPosition(240);
        tiltMotor.setPower(0.4);
        liftMotor.setPower(0.5);
        while(tiltMotor.isBusy() || liftMotor.isBusy()) {
            //wait
        }
        intakeSlides.setTargetPosition(750);
        intakeSlides.setPower(0.5);
        while(intakeSlides.isBusy()) {
            //wait
        }
        transferBeltServo.setPosition(0.52);
        yawServo.setPosition(0.32);
        double currTime = getRuntime();
        while(getRuntime() - currTime < 1) {
            //wait
        }
        pitchServo.setPosition(0.0);
        currTime = getRuntime();
        while(getRuntime() - currTime < 1) {
            //wait
        }
        intakeSlides.setTargetPosition(330);
        intakeSlides.setPower(0.5);
        while(intakeSlides.isBusy()) {
            //wait
        }
        tiltMotor.setTargetPosition(0);
        liftMotor.setTargetPosition(transferTicks);
        tiltMotor.setPower(0.5);
        liftMotor.setPower(0.5);

        while(tiltMotor.isBusy() || liftMotor.isBusy()) {
            //wait
        }

        pitchServo.getController().pwmDisable(); //disables all the servos
        transferBeltServo.getController().pwmDisable();
    }
}
