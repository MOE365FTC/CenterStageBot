package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.rr.util.Encoder;

@Config
public class Chassis {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public DcMotorEx leftOdo, strafeOdo, rightOdo;

    Gamepad gamepad1;
    IMU imu;
    LinearOpMode opMode;
    double headingOffset = 0;
    double driveSpeed = 1.0, scaleFactor;
    //Teleop Constructor
    public Chassis(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;

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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
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

        double botHeading = Math.toRadians(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

//        scaleFactor = driveSpeed + gamepad1.right_trigger * (1 - driveSpeed);
        frontLeftMotor.setPower(frontLeftPower * driveSpeed);
        backLeftMotor.setPower(backLeftPower * driveSpeed);
        frontRightMotor.setPower(frontRightPower * driveSpeed);
        backRightMotor.setPower(backRightPower * driveSpeed);
    }

    public void odoTelemetry(Telemetry telemetry){
        telemetry.addData("leftOdo", leftOdo.getCurrentPosition());
        telemetry.addData("rightOdo", rightOdo.getCurrentPosition());
        telemetry.addData("strafeOdo", strafeOdo.getCurrentPosition());

    }

    public void imuTelemetry(Telemetry telemetry) {
        telemetry.addData("imuHeading", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}