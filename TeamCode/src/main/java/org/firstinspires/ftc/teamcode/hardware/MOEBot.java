package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


public class MOEBot {
    public Chassis chassis;
    public VisionTensorflow visionTensorflow;
    public VisionBlob visionBlob;
    public DroneLauncher droneLauncher;
    public Arm arm;
    public Outtake outtake;
    public Intake intake;

    List<LynxModule> allHubs;
    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuton, boolean onRightSide) {
        // configure hubs
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new Chassis(hardwareMap, gamepad1);
        arm = new Arm(hardwareMap, gamepad1, gamepad2, telemetry, isAuton);
        visionTensorflow = new VisionTensorflow(telemetry, hardwareMap, onRightSide);
        visionBlob = new VisionBlob(telemetry, hardwareMap);
        droneLauncher = new DroneLauncher(hardwareMap, gamepad1);
//        intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry, isAuton);
//        outtake = new Outtake(hardwareMap, gamepad1, gamepad2, telemetry, isAuton);
    }

}
