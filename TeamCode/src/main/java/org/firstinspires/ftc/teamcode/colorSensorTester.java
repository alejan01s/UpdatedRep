package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by aleja on 3/26/2017.
 */

@TeleOp(name = "color tester", group = "Linear OpMode")
public class colorSensorTester extends LinearOpMode {

    colorSensorReader color;

    public void initializeRobot () {

    }

    public void runOpMode () {

        initializeRobot();

        color = new colorSensorReader("ColorL","ColorR", hardwareMap);

        while(!isStarted()){
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()){
            double[] colors = color.getColor();

            double colorLRed = colors[0];
            double colorLBlue = colors[1];
            double colorRRed = colors[2];
            double colorRBlue = colors[3];

            telemetry.addData("Color L Red: ", colorLRed);
            telemetry.addData("Color L Blue: ", colorLBlue);
            telemetry.addData("Color R Red: ", colorRRed);
            telemetry.addData("Color R Blue: ", colorRBlue);
            telemetry.update();
        }
    }
}
