package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import java.net.CacheRequest;

@TeleOp
public class GateServoTest extends OpMode {
    boolean oldGateClicked = false;
    CRServo gate;
    double gatePower = 0.5;
    boolean runGate = false;
    @Override
    public void init() {
        gate = hardwareMap.get(CRServo.class, "gate");
    }

    @Override
    public void loop() {
        telemetry.addData("Limit Switch", getPretendLimitSwitchState());
        telemetry.addData("Old Gate", oldGateClicked);
        if(gamepad1.b) {
            runGate = true;
        }
        if(runGate) {
            gate.setPower(gatePower);
            if(getPretendLimitSwitchState() && !oldGateClicked) {
                gate.setPower(0);
                runGate = false;
            }
            oldGateClicked = getPretendLimitSwitchState();
        }
    }

    private boolean getPretendLimitSwitchState() {
        return gamepad1.a;
    }
}
