package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "test")
public class colorSensor extends OpMode {
    RevColorSensorV3 colorSensorFront;
    RevColorSensorV3 colorSensorBack;

    RevBlinkinLedDriver frontBlinkin, backBlinkin;

    //yellow: ranges: (500-1000. 700-1300, 50-500), rgb: (650, 890, 220)
    //green: ranges:  (0-500, 350-950, 0-500), rgb: (210, 630, 210)
    //purple: ranges: (200-800, 500-1100, 500-1000), rgb:
    //white: ranges: (600-1400, 1400-1900, 900-1350),
    @Override
    public void init() {
        colorSensorFront = hardwareMap.get(RevColorSensorV3.class, "CSF");
        colorSensorBack = hardwareMap.get(RevColorSensorV3.class, "CSB");
        frontBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkFront");
        backBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkBack");
    }

    @Override
    public void loop() {
        telemetry.addData("F - red: ",colorSensorFront.red());
        telemetry.addData("F - green: ",colorSensorFront.green());
        telemetry.addData("F - blue: ",colorSensorFront.blue());

        telemetry.addData("B - red: ",colorSensorBack.red());
        telemetry.addData("B - green: ",colorSensorBack.green());
        telemetry.addData("B - blue: ",colorSensorBack.blue());

//        telemetry.addData("F - yellow pixel: ", testFront(500,1000,700, 1300, 50, 500));
//        telemetry.addData("F - green pixel: ", testFront(0,500,350,950,0,500));
//        telemetry.addData("F - purple pixel: ", testFront(200,800,350,950,500,1000));
//        telemetry.addData("F - white pixel: ", testFront(1000,1600, 2000,2500,1200,1700));
        telemetry.addData("F - Pixel", newTestFront());

//        telemetry.addData("B - yellow pixel: ", testBack(500,1000,700, 1300, 50, 500));
//        telemetry.addData("B - green pixel: ", testBack(0,500,350,950,0,500));
//        telemetry.addData("B - purple pixel: ", testBack(200,800,350,950,500,1000));
//        telemetry.addData("B - white pixel: ", testBack(1000,1600, 2000,2500,1200,1700));
        telemetry.addData("B - Pixel", newTestBack());

    }

    public boolean testFront(int lowR, int highR, int lowG, int highG, int lowB, int highB) {
        return lowR < colorSensorFront.red() && colorSensorFront.red() < highR && lowG < colorSensorFront.green() && colorSensorFront.green() < highG && lowB < colorSensorFront.blue() && colorSensorFront.blue() < highB;
    }

    public boolean testBack(int lowR, int highR, int lowG, int highG, int lowB, int highB) {
        return lowR < colorSensorBack.red() && colorSensorBack.red() < highR && lowG < colorSensorBack.green() && colorSensorBack.green() < highG && lowB < colorSensorBack.blue() && colorSensorBack.blue() < highB;
    }

    public String newTestFront() {
        String str = "";
        if(testFront(100,1000000,100, 1000000, 100, 1000000)) {
            str = "Pixel In";
            frontBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            str = "None";
            frontBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        return str;
    }

    public String newTestBack() {
        String str = "";
        if(testBack(100,1000000,100, 1000000, 100, 1000000)) {
            str = "Pixel In";
            backBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            str = "None";
            backBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        return str;
    }
}
