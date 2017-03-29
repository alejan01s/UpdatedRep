package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by aleja on 3/24/2017.
 */

@TeleOp(name="Sonar Tester", group="Linear Opmode")
public class sonarReaderTester extends LinearOpMode {

    sonarReader sonar;

    public void initializeRobot (){

    }

    public void runOpMode () throws InterruptedException {

        initializeRobot();

        sonar = new sonarReader("SonarL", "SonarR", hardwareMap);
        sleep(200);

        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()){
            double[] distances = sonar.getDistances();

            double distanceLeft = distances[0];
            double distanceRight = distances[1];

            telemetry.addData("Left Distance: ", distanceLeft);
            telemetry.addData("Right Distance: ", distanceRight);
            telemetry.update();
            idle();
        }
    }
}
