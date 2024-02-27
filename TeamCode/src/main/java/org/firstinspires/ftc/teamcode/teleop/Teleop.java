package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.hardware.PoseStorage;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void init_loop(){
        robot.chassis.imuTelemetry(telemetry);
    }

    @Override
    public void loop() {
        robot.chassis.imuTelemetry(telemetry);
        robot.chassis.odoTelemetry(telemetry);
        robot.chassis.fieldCentricDrive();
        robot.outtake.actuate();
        robot.intake.updateGates();
        robot.droneLauncher.actuate();
        robot.hang.actuate();

        if(gamepad2.b) {
            robot.intake.runGates();
        }
    }
}
