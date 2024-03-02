package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MOEBot {
    public Chassis chassis;
    public VisionTensorflow visionTensorflow;
    public VisionBlob visionBlob;
    public DroneLauncher droneLauncher;
    public Intake intake;
//    public DispenserDec17 dispenser;
    public Outtake outtake;
    public Hanging hang;

    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton) {
        chassis = new Chassis(hardwareMap, gamepad1);
//        visionTensorflow = new VisionTensorflow(telemetry, hardwareMap);
//        visionBlob = new VisionBlob(telemetry, hardwareMap);
//        droneLauncher = new DroneLauncher(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry, isAuton);
        outtake = new Outtake(hardwareMap, gamepad1, gamepad2, telemetry, isAuton);
    }

}
