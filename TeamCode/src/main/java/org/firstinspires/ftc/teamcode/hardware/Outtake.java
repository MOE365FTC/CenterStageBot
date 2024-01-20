package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake {
    private PIDController controller;
    Servo leftIris, rightIris, pitchServo;
    DcMotor extensionMotor, tiltMotor; //extensionMotor controls the length of arm, tiltMotor controls rotation/angle of the arm

    //presets
    double irisExpand = 0.2, irisContract = 0, intakePitch = 1, scorePitch = 0.68; //defaults
    int backboardTilt; //tuning needed; angle of backboard is ~ 60 degrees(arm wont be be pressed right on the back board)
    double extendPower; //tuning needed
    double tiltPower; //tuning needed
    double extendStickThreshhold = 0.3; //
    double extendResetPos, tiltResetPos, pitchResetPos;

    //pidf rot arm
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;

    private final double ticks_in_degree=700/180;//needs calibration



    //status
    boolean leftClosed = false, rightClosed = false, scheduledOpen = false;
    boolean oldLeftBumper = false, oldRightBumper = false;
    boolean stageLock = false; //status if locked onto stage


    Gamepad gamepad2;
    Telemetry telemetry;
    public Outtake(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry){
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        leftIris = hardwareMap.get(Servo.class, "leftIris");
        rightIris = hardwareMap.get(Servo.class, "rightIris");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        extensionMotor = hardwareMap.get(DcMotor.class, "extend");
        tiltMotor = hardwareMap.get(DcMotor.class, "extend");

        leftIris.setPosition(irisExpand);
        rightIris.setPosition(irisExpand);
        pitchServo.setPosition(intakePitch);

        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        controller = new PIDController(p,i,d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void actuate() {
        //iris

        if(gamepad2.left_bumper)
            leftIris.setPosition(irisContract);
        else
            leftIris.setPosition(irisExpand);

        if(gamepad2.right_bumper)
            rightIris.setPosition(irisContract);
        else
            rightIris.setPosition(irisExpand);

        //extend arm

        if (-gamepad2.left_stick_y > 0.75)
            extensionMotor.setPower(extendPower);
        else if (-gamepad2.left_stick_y < -0.75)
            extensionMotor.setPower(-extendPower);
        else
            extensionMotor.setPower(0);

        //rot arm

        if (-gamepad2.right_stick_y > 0.75)
            tiltMotor.setPower(tiltPower);
        else if (-gamepad2.right_stick_y < -0.75)
            tiltMotor.setPower(-tiltPower);
        else
            tiltMotor.setPower(0);

        //pidf rot arm

        controller.setPID(p,i,d);
        int armPos = tiltMotor.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        tiltPower = pid + ff;

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();



    }
    private void extendArm(int targetPos) {
        extensionMotor.setTargetPosition(targetPos);
        extensionMotor.setPower(extendPower);
    }




    public void autonIris(boolean expand) { //auton method for iris control
        leftIris.setPosition(expand ? irisExpand : irisContract);
        rightIris.setPosition(expand ? irisExpand : irisContract);
    }

    public void autonLeftIris(boolean expand) { //individual auton iris control
        leftIris.setPosition(expand ? irisExpand : irisContract);
    }

    public void autonRightIris(boolean open) { //individual auton iris control
        rightIris.setPosition(open ? irisExpand : irisContract);
    }
}
