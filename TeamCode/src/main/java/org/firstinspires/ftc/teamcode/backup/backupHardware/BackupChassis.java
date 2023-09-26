package org.firstinspires.ftc.teamcode.backup.backupHardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BackupChassis {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public DcMotorEx leftOdo, strafeOdo, rightOdo;

    Gamepad gamepad1;
    LinearOpMode opMode;
    BackupIMU imu;
    double headingOffset = 0;
    double driveSpeed = 1.0, scaleFactor;
//    SampleMecanumDrive drive = new SampleMecanumDrive(opMode.hardwareMap);
    //Teleop Constructor
    public BackupChassis(HardwareMap hardwareMap, BackupIMU imu, Gamepad gamepad1){
     this.gamepad1 = gamepad1;
     this.imu = imu;

     frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM02");
     backLeftMotor = hardwareMap.get(DcMotor.class, "BLM00");
     frontRightMotor = hardwareMap.get(DcMotor.class, "FRM03");
     backRightMotor = hardwareMap.get(DcMotor.class, "BRM01");

     leftOdo = hardwareMap.get(DcMotorEx.class, "OLE11");
     strafeOdo = hardwareMap.get(DcMotorEx.class, "OCE12");
     rightOdo = hardwareMap.get(DcMotorEx.class, "ORE13");

//     backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//     frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//     frontLeftMotor.setTargetPosition(0);
//     backLeftMotor.setTargetPosition(0);
//     frontRightMotor.setTargetPosition(0);
//     backRightMotor.setTargetPosition(0);

//     frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void fieldCentricDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = Math.toRadians(-imu.getHeadingFirstAngle());

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

//        scaleFactor = driveSpeed + gamepad1.right_trigger * (1 - driveSpeed);
        scaleFactor = driveSpeed;
        frontLeftMotor.setPower(frontLeftPower * scaleFactor);
        backLeftMotor.setPower(backLeftPower * scaleFactor);
        frontRightMotor.setPower(frontRightPower * scaleFactor);
        backRightMotor.setPower(backRightPower * scaleFactor);
    }

    public void odoTelemetry(Telemetry telemetry){
        telemetry.addData("leftOdo", leftOdo.getCurrentPosition());
        telemetry.addData("rightOdo", rightOdo.getCurrentPosition());
        telemetry.addData("strafeOdo", strafeOdo.getCurrentPosition());

    }
}
