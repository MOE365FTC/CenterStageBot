package org.firstinspires.ftc.teamcode.hardware;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneLauncher {
    Servo flywheel;
    Gamepad gamepad1;
    public DroneLauncher(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;

        flywheel = hardwareMap.get(Servo.class, "FLY");
        flywheel.setPosition(0.0);
    }
    public void actuate(){if (gamepad1.dpad_up){flywheel.setPosition(0.5);}}
}
