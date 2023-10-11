package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MOEBot {
    public Chassis chassis;
    public VisionTensorflow vision;

    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
//        chassis = new Chassis(hardwareMap, gamepad1);
        telemetry.addLine("hardware");
        telemetry.update();
        vision = new VisionTensorflow(telemetry, hardwareMap);
    }

}
