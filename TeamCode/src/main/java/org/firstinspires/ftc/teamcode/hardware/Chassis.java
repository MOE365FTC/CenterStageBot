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
    static IMU imu; //A static IMU object that shares heading between TeleOp and Auton
    double headingOffset = 0; //stores the heading the robot started the opmode with (corrects for error)
    double driveSpeed = 1.0;

    //Teleop Constructor
    public Chassis(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        leftOdo = hardwareMap.get(DcMotorEx.class, "FLM"); //odo encoders are on motor ports CHECK
        rightOdo = hardwareMap.get(DcMotorEx.class, "FRM");
        strafeOdo = hardwareMap.get(DcMotorEx.class, "BLM");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void fieldCentricDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) { //resets field-centric drive heading (offset = current heading)
            headingOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        double botHeading = Math.toRadians(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if(gamepad1.right_bumper) //slow down button for fine-control
            driveSpeed = 0.5;
        else
            driveSpeed = 1;
        
        frontLeftMotor.setPower(frontLeftPower * driveSpeed);
        backLeftMotor.setPower(backLeftPower * driveSpeed);
        frontRightMotor.setPower(frontRightPower * driveSpeed);
        backRightMotor.setPower(backRightPower * driveSpeed);
    }

    public void odoTelemetry(Telemetry telemetry){
        telemetry.addData("leftOdo", leftOdo.getCurrentPosition());
        telemetry.addData("rightOdo", rightOdo.getCurrentPosition());
        telemetry.addData("strafeOdo", strafeOdo.getCurrentPosition());

        telemetry.addData("FRM", frontRightMotor.getPower());

    }

    public void imuTelemetry(Telemetry telemetry) {
        telemetry.addData("imuHeading", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset);
    }
}