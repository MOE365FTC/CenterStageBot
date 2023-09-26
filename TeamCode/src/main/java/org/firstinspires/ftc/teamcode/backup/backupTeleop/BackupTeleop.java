package org.firstinspires.ftc.teamcode.backup.backupTeleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.backup.backupHardware.BackupMOEBot;

@TeleOp
public class BackupTeleop extends OpMode {
    BackupMOEBot robot;
    @Override
    public void init() {
        robot = new BackupMOEBot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void init_loop(){
        telemetry.addData("headingIMU" ,robot.backupimu.getHeadingFirstAngle());
    }

    @Override
    public void loop() {
        robot.backuplift.actuate();
        robot.backuplift.lowerLift();
        robot.backupClaw.actuate();
        robot.backupChassis.fieldCentricDrive();
//        robot.chassis.drive();
        robot.backuplift.forkActuate();
        if(gamepad1.a){
            robot.backupClaw.grab();
        }
        robot.backupChassis.odoTelemetry(telemetry);
        telemetry.addData("lift", robot.backuplift.getLiftTicks());
        telemetry.update();
    }
}