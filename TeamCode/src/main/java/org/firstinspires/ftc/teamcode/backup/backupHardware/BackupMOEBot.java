package org.firstinspires.ftc.teamcode.backup.backupHardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class BackupMOEBot {
    public BackupChassis backupChassis;
    public BackupClaw backupClaw;
    public BackupIMU backupimu;
    public BackupLift backuplift;

    //TeleOp Constructor
    public BackupMOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        backupimu = new BackupIMU(hardwareMap);
        backupChassis = new org.firstinspires.ftc.teamcode.backup.backupHardware.BackupChassis(hardwareMap, backupimu, gamepad1);
        backupClaw = new BackupClaw(hardwareMap, gamepad1, gamepad2);
        backuplift = new BackupLift(hardwareMap, gamepad1, gamepad2);
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