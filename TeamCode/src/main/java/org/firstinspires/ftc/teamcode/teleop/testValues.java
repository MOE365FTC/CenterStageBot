package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.hardware.Outtake;

@TeleOp
public class testValues extends OpMode {
    MOEBot robot;
//    Servo leftIris, rightIris;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, telemetry, false);
    }

    @Override
    public void init_loop(){
        robot.chassis.imuTelemetry(telemetry);
    }

    @Override
    public void loop() {
//        robot.chassis.fieldCentricDrive();
        robot.chassis.imuTelemetry(telemetry);
        robot.chassis.odoTelemetry(telemetry);
        robot.outtake.telemetryOuttake();
        robot.intake.telemetryIntake();

        telemetry.addData("grabLeftSwitch", robot.intake.grabLeftSwitch.getState());
        telemetry.addData("grabRightSwitch", robot.intake.grabRightSwitch.getState());
        switch(Intake.currExtendPos) {
            case BASE:
                telemetry.addData("CurrentExtendPositions", "BASE");
                break;
            case TRANSFER:
                telemetry.addData("CurrentExtendPositions", "TRANSFER");
                break;
            case EXTENDED_FULL:
                telemetry.addData("CurrentExtendPositions", "EXTENDED_FULL");
        }
        switch(Intake.currRequestPos) {
            case BASE:
                telemetry.addData("RequestedExtendPositions", "BASE");
                break;
            case TRANSFER:
                telemetry.addData("RequestedExtendPositions", "TRANSFER");
                break;
            case EXTENDED_FULL:
                telemetry.addData("RequestedExtendPositions", "EXTENDED_FULL");
                break;
        }
        switch(Outtake.currTiltPos) {
            case READY_TO_OUTTAKE:
                telemetry.addData("CurrentOuttakeState", "UP");
                break;
            case READY_TO_INTAKE:
                telemetry.addData("CurrentOuttakeState", "DOWN");
                break;
        }

        if(gamepad1.right_trigger > 0.7){
            robot.outtake.autonIris(true);
        }else if(gamepad1.left_trigger > 0.7){
            robot.outtake.autonIris(false);
        }

//        robot.outtake.actuate();
//        robot.intake.actuate();
//        robot.intake.updateGrabs();
    }
}
