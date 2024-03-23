package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(group = "test")
public class colorSensor extends OpMode {
    ColorSensor colorSensor;

    //yellow: ranges: (500-1000. 700-1300, 50-500), rgb: (650, 890, 220)
    //green: ranges:  (0-500, 350-950, 0-500), rgb: (210, 630, 210)
    //purple: ranges: (200-800, 500-1100, 500-1000), rgb:
    //white: ranges: (600-1400, 1400-1900, 900-1350),
    @Override
    public void init() {colorSensor = hardwareMap.get(ColorSensor.class, "CLS");}

    @Override
    public void loop() {
        telemetry.addData("red: ",colorSensor.red());
        telemetry.addData("green: ",colorSensor.green());
        telemetry.addData("blue: ",colorSensor.blue());

        telemetry.addData("yellow pixel: ", test(500,1000,700, 1300, 50, 500));
        telemetry.addData("green pixel: ", test(0,500,350,950,0,500));
        telemetry.addData("purple pixel: ", test(200,800,350,950,500,1000));
        telemetry.addData("white pixel: ", test(1000,1600, 2000,2500,1200,1700));
    }

    public boolean test(int lowR, int highR, int lowG, int highG, int lowB, int highB) {
        return lowR < colorSensor.red() && colorSensor.red() < highR && lowG < colorSensor.green() && colorSensor.green() < highG && lowB < colorSensor.blue() && colorSensor.blue() < highB;
    }
}
