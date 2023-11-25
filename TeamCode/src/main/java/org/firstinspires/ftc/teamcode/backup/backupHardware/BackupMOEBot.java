package org.firstinspires.ftc.teamcode.backup.backupHardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.VisionTensorflow;


public class BackupMOEBot {
    public BackupChassis backupChassis;
    public BackupClaw backupClaw;
    public BackupIMU backupimu;
    public BackupLift backuplift;
    public VisionTensorflow vision;

    //Original TeleOp Constructor for PowerPlay
    public BackupMOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        backupimu = new BackupIMU(hardwareMap);
        backupChassis = new org.firstinspires.ftc.teamcode.backup.backupHardware.BackupChassis(hardwareMap, backupimu, gamepad1);
        backupClaw = new BackupClaw(hardwareMap, gamepad1, gamepad2);
        backuplift = new BackupLift(hardwareMap, gamepad1, gamepad2);
    }

    //Updated TeleOp Constructor for PowerPlay w/ Vision
    public BackupMOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        backupimu = new BackupIMU(hardwareMap);
        backupChassis = new org.firstinspires.ftc.teamcode.backup.backupHardware.BackupChassis(hardwareMap, backupimu, gamepad1);
        backupClaw = new BackupClaw(hardwareMap, gamepad1, gamepad2);
        backuplift = new BackupLift(hardwareMap, gamepad1, gamepad2);
        vision = new VisionTensorflow(telemetry, hardwareMap);
    }

    //Autonomous Constructor
//    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1) {
//        Backupimu = new BackupIMU(hardwareMap);
//        chassis = new Chassis(hardwareMap, Backupimu, gamepad1);
//        BackupClaw = new BackupClaw();
//        turret = new Turret(hardwareMap, Backupimu, gamepad1);
//        vision = new Vision(hardwareMap);
//    }
}