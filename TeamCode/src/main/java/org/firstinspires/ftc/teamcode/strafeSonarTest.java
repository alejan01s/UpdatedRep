package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aleja on 3/24/2017.
 */

@TeleOp(name="Strafe Test", group="Linear OpMode")
public class strafeSonarTest extends LinearOpMode {

    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    boolean runLeft;
    boolean runRight;

    sonarReader sonar;

    public void initializeRobot (){

        FR = hardwareMap.dcMotor.get("Fr");
        FL = hardwareMap.dcMotor.get("Fl");
        BR = hardwareMap.dcMotor.get("Br");
        BL = hardwareMap.dcMotor.get("Bl");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

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
            double[] distances = sonar.getDistances("left");

            double distanceLeft = distances[0];
            double distanceRight = distances[1];

            telemetry.addData("Left Distance: ", distanceLeft);
            telemetry.addData("Right Distance: ", distanceRight);
            telemetry.update();

            if(gamepad1.a) {
                runLeft = true;
            }

            if(runLeft){
                if (distanceLeft > 40 || distanceLeft == 0) {
                    FR.setPower(-.25);
                    BR.setPower(.25);
                    FL.setPower(.25);
                    BL.setPower(-.25);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    telemetry.addData("Successfully reached left distance and stopped.", "");
                    runLeft = false;
                }
            }

            if(gamepad1.b){
                runRight = true;
            }

            if(runRight){
                if (distanceRight > 40 || distanceRight == 0) {
                    FR.setPower(.25);
                    BR.setPower(-.25);
                    FL.setPower(-.25);
                    BL.setPower(.25);
                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    telemetry.addData("Successfully reached right distance and stopped.", "");
                    runRight = false;
                }
            }
        }
    }
}
