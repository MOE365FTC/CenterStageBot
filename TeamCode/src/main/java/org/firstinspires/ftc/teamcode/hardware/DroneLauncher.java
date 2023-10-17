package org.firstinspires.ftc.teamcode.hardware;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class DroneLauncher {
    DcMotor flywheel;
    Gamepad gamepad1;
    public DroneLauncher(HardwareMap hardwareMap, Gamepad gamepad1){
        this.gamepad1 = gamepad1;

        flywheel = hardwareMap.get(DcMotor.class, "FLY");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void actuate(){if (gamepad1.dpad_up){flywheel.setPower(0.75);}}
}
