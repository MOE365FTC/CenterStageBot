package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MOEBot {
    public Chassis chassis;

    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        chassis = new Chassis(hardwareMap, gamepad1);
    }

}
