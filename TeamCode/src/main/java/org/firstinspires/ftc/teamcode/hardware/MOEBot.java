package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MOEBot {
    public Chassis chassis;
    public VisionTensorflow vision;
    public DroneLauncher droneLauncher;
    public Intake intake;
    public Dispenser dispenser;

    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        chassis = new Chassis(hardwareMap, gamepad1);
        vision = new VisionTensorflow(telemetry, hardwareMap);
        droneLauncher = new DroneLauncher(hardwareMap, gamepad1);
        dispenser = new Dispenser(hardwareMap, gamepad2, telemetry);
//        intake = new Intake(hardwareMap, gamepad1, gamepad2, dispenser);

    }

}
