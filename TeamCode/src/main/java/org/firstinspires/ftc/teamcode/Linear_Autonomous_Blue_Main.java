package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by aleja on 3/26/2017.
 */

@Autonomous(name = "Blue Center Autonomous", group = "Sensors")
public class Linear_Autonomous_Blue_Main extends LinearOpMode {

    //MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    //VARIABLES FOR LAUNCHER
    public double EncoderClicks = 2510;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public DcMotor LauncherM;
    public Servo Reloader;

    public boolean turnCompleted = false;

    public OpticalDistanceSensor bottomOD;
    public OpticalDistanceSensor frontOD;
    public OpticalDistanceSensor colorOD;

    public Servo BallG1;
    public Servo BallG2;

    public boolean isRed;
    public boolean isBlue;

    //BUTTON PUSHER
    public Servo buttonPusher;
    public Servo buttonPusher2;

    public boolean buttonPress;
    public boolean buttonInit;
    public boolean push;

    public double NumberOfRevs5 = 0;
    public double NumberOfRevs4;
    public double NumberOfRevs3;

    public double colorCheckStep = 0;

    public boolean pushed = false;
    public boolean OppPushSequence = false;
    public boolean nearPush = false;
    public boolean shoot1;
    public boolean fired;

    public double numRevs;

    public boolean encoderTurn = false;

    public void initializeRobot() {

        //DRIVE-TRAIN MOTORS
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");
        FL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);


        //LAUNCHER MOTORS
        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");
        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //SENSORS
        bottomOD = hardwareMap.opticalDistanceSensor.get("bottomOD");
        frontOD = hardwareMap.opticalDistanceSensor.get("backOD");
        colorOD = hardwareMap.opticalDistanceSensor.get("frontOD");

        //BUTTON PUSHER

        buttonPusher = hardwareMap.servo.get("buttonPusher2");
        buttonPusher2 = hardwareMap.servo.get("buttonPusher");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");

        buttonInit = false;
        buttonPress = false;
        push = false;
    }

    public void runOpMode() throws InterruptedException {

        initializeRobot();

        //SEQUENCE VARIABLE
        double step = 0;

        //INITIAL REVOLUTION VARIABLES
        int NumberOfRevs1 = -700;
        int NumberOfRevs2 = -200;

        imuTest imu = new imuTest("imu", hardwareMap);
        sonarReader sonar = new sonarReader("SonarL", "SonarR", hardwareMap);
        colorSensorReader color = new colorSensorReader("ColorL", "ColorR", hardwareMap);

        while (!isStarted()) {
            telemetry.addData("Status: ", "Initialization Complete"); //DRIVER PROGRAMS INITIALIZED
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {

            //GET VALUES FROM DRIVERS

            //IMU
            double[] angles = imu.getAngles();
            double yaw = angles[0];
            double x = yaw;

            if (x < 0) {
                x = x + 360;
            }

            //SONAR
            double[] distances = sonar.getDistances("right");

            double distanceLeft = distances[0];
            double distanceRight = distances[1];

            //COLOR SENSORS
            double[] colors = color.getColor();

            double colorLRed = colors[0];
            double colorLBlue = colors[1];
            double colorRRed = colors[2];
            double colorRBlue = colors[3];

            //DETECT COLOR BOOLEANS
            isRed = colorRRed >= 1 && colorRRed > colorRBlue ? true : false;
            isBlue = colorRBlue >= 1 && colorRBlue > colorRRed ? true : false;

            //REQUIRED SERVO ZEROES
            BallG1.setPosition(0);
            BallG2.setPosition(1);
            buttonPusher2.setPosition(.5);

            //PROGRAM STEPS
            if (step == 0) {
                if (FL.getCurrentPosition() > NumberOfRevs1) {
                    BL.setPower(-.25);
                    BR.setPower(-.25);
                    FR.setPower(-.25);
                    FL.setPower(-.25);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }
            if (step == 1) {
                if (!fired) {
                    shoot1 = true;
                }
                if (fired) {
                    step = step + 1;
                }
            }
            if (step == 2) {
                if (!shoot) {
                    sleep(250);
                    shoot = true;
                    step = step + 1;
                }
            }
            if (step == 3) {
                if (!shoot) {
                    if (FL.getCurrentPosition() < NumberOfRevs2) {
                        BL.setPower(.25);
                        BR.setPower(.25);
                        FR.setPower(.25);
                        FL.setPower(.25);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        NumberOfRevs3 = FL.getCurrentPosition() - 2000;
                        step = step + .25;
                    }
                }
            }
            if(step == 3.25){
                if (FL.getCurrentPosition() > NumberOfRevs3) {
                    FR.setPower(0);
                    BR.setPower(-1);
                    FL.setPower(-1);
                    BL.setPower(0);
                } else {
                    step = step + .25;
                }
            }
            if (step == 3.5) {
                if (distanceRight > 50 || distanceRight == 0) {
                    sonar.getDistances("right");
                    distanceRight = distances[1];
                    FR.setPower(0);
                    BR.setPower(-.75);
                    FL.setPower(-.75);
                    BL.setPower(0);
                } else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    sonar.resetRange(6, 6);
                    step = step + .5;
                }
            }
            if (step == 4) {
//                if (yaw > -1.5 && yaw < 0) {
//                    //has reached angle therefore end loop
//                    FR.setPower(0);
//                    FL.setPower(0);
//                    BR.setPower(0);
//                    BL.setPower(0);
//                    step = step + 2;
//                } else if (yaw < -1.5) {
//                    //turn clockwise
//                    FR.setPower(-.2);
//                    FL.setPower(.2);
//                    BR.setPower(-.2);
//                    BL.setPower(.2);
//                } else if (yaw > 0) {
//                    //turn counter-clockwise
//                    FR.setPower(0.2);
//                    FL.setPower(-.2);
//                    BR.setPower(0.2);
//                    BL.setPower(-.2);
//                }
                while(yaw < 2 || yaw > 3){
                    angles = imu.getAngles();
                    yaw = angles[0];
                    if(yaw < 2){

                        //turn clockwise
                        FR.setPower(-.1);
                        FL.setPower(.1);
                        BR.setPower(-.1);
                        BL.setPower(.1);

                    }
                    else if(yaw > 3){

                        //turn counter-clockwise
                        FR.setPower(0.1);
                        FL.setPower(-.1);
                        BR.setPower(0.1);
                        BL.setPower(-.1);

                    }

                    if(yaw > 2 && yaw < 3)
                    {

                        break;

                    }
                }

                //has reached angle therefore end loop
                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                step=step+1;
            }
            if(step == 5){
                while(bottomOD.getRawLightDetected() < .04){
                    FR.setPower(.25);
                    FL.setPower(.25);
                    BR.setPower(.25);
                    BL.setPower(.25);
                }
                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                step = step + 1;
            }
            if (step == 6) {
                if (distanceRight > 15 || distanceRight == 0) {
                    sonar.getDistances("right");
                    distanceRight = distances[1];
                    FR.setPower(.1);
                    BR.setPower(-.1);
                    FL.setPower(-.1);
                    BL.setPower(.1);
                } else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    step = step + .5;
                }
            }
            if (step == 6.5) {
                NumberOfRevs3 = FL.getCurrentPosition() + 35;
                step = step + .5;
            }
            if (step == 6.75) {
                if (FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(.25);
                    BR.setPower(.25);
                    FR.setPower(.25);
                    FL.setPower(.25);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + .25;
                }
            }
            if (step == 7) {
                numRevs = FL.getCurrentPosition() - 105;
                NumberOfRevs3 = FL.getCurrentPosition() - 450;
                step = step + 1;
            }
            if (step == 8) {
                isRed = colorRRed >= 1 && colorRRed > colorRBlue ? true : false;
                isBlue = colorRBlue >= 1 && colorRBlue > colorRRed ? true : false;
                if (isBlue && !OppPushSequence) {
                    //push button
                    nearPush = true;
                } else if (isRed && !nearPush) {
                    //move forward confirm and push button
                    OppPushSequence = true;
                }
                if (nearPush) {
                    if (FL.getCurrentPosition() > numRevs) {
                        BL.setPower(-.1);
                        BR.setPower(-.1);
                        FR.setPower(-.1);
                        FL.setPower(-.1);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        sleep(5);
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + 1;
                        }
                    }
                } else if (OppPushSequence) {
                    if (FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.1);
                        BR.setPower(-.1);
                        FR.setPower(-.1);
                        FL.setPower(-.1);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        sleep(5);
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + 1;
                        }
                    }
                }
            }
            if (step == 9) {
                pushed = false;
                nearPush = false;
                OppPushSequence = false;
                NumberOfRevs3 = FL.getCurrentPosition() - 500;
                step = step + 1;
            }
            if(step == 10){
                if(FL.getCurrentPosition() > NumberOfRevs3) {
                    BL.setPower(-1);
                    BR.setPower(-1);
                    FR.setPower(-1);
                    FL.setPower(-1);
                }
                else {
                    step=step+1;
                }
            }
            if (step == 11) {
                while (bottomOD.getRawLightDetected() < .04) {
                    FL.setPower(-.5);
                    BL.setPower(-.5);
                    FR.setPower(-.5);
                    BR.setPower(-.5);
                }
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                step = step + .5;

            }
            if (step == 11.5) {
                while(yaw < 2 || yaw > 3){
                    angles = imu.getAngles();
                    yaw = angles[0];
                    if(yaw < 2){

                        //turn clockwise
                        FR.setPower(-.1);
                        FL.setPower(.1);
                        BR.setPower(-.1);
                        BL.setPower(.1);

                    }
                    else if(yaw > 3){

                        //turn counter-clockwise
                        FR.setPower(0.1);
                        FL.setPower(-.1);
                        BR.setPower(0.1);
                        BL.setPower(-.1);

                    }

                    if(yaw > 2 && yaw < 3)
                    {

                        break;

                    }
                }

                //has reached angle therefore end loop
                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                NumberOfRevs3 = FL.getCurrentPosition() + 10;
                step=step+.5;
            }
            if (step == 12) {
                if (FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(.1);
                    BR.setPower(.1);
                    FR.setPower(.1);
                    FL.setPower(.1);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }
            if (step == 13) {
                numRevs = FL.getCurrentPosition() - 45;
                NumberOfRevs3 = FL.getCurrentPosition() - 340;
                step = step + 1;
            }
            if (step == 14) {
                if (distanceRight > 15 || distanceRight == 0) {
                    sonar.getDistances("right");
                    distanceRight = distances[1];
                    FR.setPower(.1);
                    BR.setPower(-.1);
                    FL.setPower(-.1);
                    BL.setPower(.1);
                } else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    numRevs = FL.getCurrentPosition() - 90;
                    NumberOfRevs3 = FL.getCurrentPosition() - 375;
                    step = step + 1;
                }
            }
            if (step == 15) {
                isRed = colorRRed >= 1 && colorRRed > colorRBlue ? true : false;
                isBlue = colorRBlue >= 1 && colorRBlue > colorRRed ? true : false;
                if (isBlue && !OppPushSequence) {
                    //push button
                    nearPush = true;
                } else if (isRed && !nearPush) {
                    //move forward confirm and push button
                    OppPushSequence = true;
                }
                if (nearPush) {
                    if (FL.getCurrentPosition() > numRevs) {
                        BL.setPower(-.1);
                        BR.setPower(-.1);
                        FR.setPower(-.1);
                        FL.setPower(-.1);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        sleep(5);
                        if (!pushed) {
                            push = true;
                        } else {
                            step = step + .5;
                        }
                    }
                } else if (OppPushSequence) {
                    if (FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.1);
                        BR.setPower(-.1);
                        FR.setPower(-.1);
                        FL.setPower(-.1);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        sleep(5);
                        if (!pushed) {
                            push = true;
                        }
                        if (pushed) {
                            step = step + .5;
                        }
                    }
                }
            }
            if (step == 15.5) {
                FR.setPower(-.5);
                BR.setPower(.5);
                FL.setPower(.5);
                BL.setPower(-.5);
                Thread.sleep(500);
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
                if(yaw == 0){
                    encoderTurn = true;
                }
                step = step + .25;
            }
            if (step == 15.75) {
                NumberOfRevs3 = FL.getCurrentPosition() - 450;
                if (OppPushSequence) {
                    NumberOfRevs3 = FL.getCurrentPosition() - 400;
                }
                step = step + .25;
            }
            if (step == 16) {
                if(!encoderTurn) {
                    while (yaw > -29) {
                        angles = imu.getAngles();
                        yaw = angles[0];

                        FR.setPower(.4);
                        FL.setPower(-.4);
                        BR.setPower(.4);
                        BL.setPower(-.4);

                    }
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);

                    step = step + 1;
                }
                else{
                    if (FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.5);
                        BR.setPower(.5);
                        FR.setPower(.5);
                        FL.setPower(-.5);
                    }
                    else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        step = step + 1;
                    }
                }
            }
            if (step == 17) {
                turnCompleted = false;
                NumberOfRevs3 = FL.getCurrentPosition() + 2900;
                step = step + 1;
            }
            if (step == 18) {
                if (FL.getCurrentPosition() < NumberOfRevs3) {
                    BL.setPower(1);
                    BR.setPower(1);
                    FR.setPower(1);
                    FL.setPower(1);
                } else {
                    BL.setPower(0);
                    BR.setPower(0);
                    FR.setPower(0);
                    FL.setPower(0);
                    step = step + 1;
                }
            }

            //BUTTON PUSHER
            if (!buttonInit) {
                buttonPusher.setPosition(.5);
                if (push) {
                    buttonInit = true;
                }
            } else {
                buttonPusher2.setPosition(0);
                sleep(1000);
                buttonPusher2.setPosition(1);
                sleep(1000);
                buttonInit = false;
                push = false;
                pushed = true;
            }

            //SHOOTER CODE
            if (shoot1) {
                if (LauncherM.getCurrentPosition() <= 2000) {
                    LauncherM.setPower(1);
                } else if (LauncherM.getCurrentPosition() <= 2450) {
                    LauncherM.setPower(.08);
                } else {
                    LauncherM.setPower(0);
                    fired = true;
                    shoot1 = false;
                    EncoderClicks = EncoderClicks + 2510;
                }
            }
            if (shoot) {

                if (LauncherM.getCurrentPosition() <= 1400 + (EncoderClicks - 2510)) {

                    Reloader.setPosition(0.65);
                    LauncherM.setPower(0.75);

                } else if (LauncherM.getCurrentPosition() <= 2250 + (EncoderClicks - 2510)) {

                    Reloader.setPosition(0.1);
                    LauncherM.setPower(1);

                } else if (LauncherM.getCurrentPosition() > 2250 + (EncoderClicks - 2510) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                    LauncherM.setPower(.1);
                } else {
                    LauncherM.setPower(0);
                    shoot = false;
                    EncoderClicks = EncoderClicks + 2510;
                }
            }
//            if (!shoot && !shoot1) {
//                if (LauncherM.getCurrentPosition() > (EncoderClicks - 2510)) {
//
//                    LauncherM.setPower(-0.07);
//
//                }
//
//                if (LauncherM.getCurrentPosition() < (EncoderClicks - 2510)) {
//
//                    LauncherM.setPower(0.07);
//
//                }
//            }

            //TELEMETRY DATA
            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("colorOD: ", colorOD.getRawLightDetected());

            telemetry.addData("Color L Red: ", colorLRed);
            telemetry.addData("Color L Blue: ", colorLBlue);
            telemetry.addData("Color R Red: ", colorRRed);
            telemetry.addData("Color R Blue: ", colorRBlue);

            telemetry.addData("Left Distance: ", distanceLeft);
            telemetry.addData("Right Distance: ", distanceRight);

            telemetry.addData(imu.getName(), imu.telemetrize());
            telemetry.addData("X: ", x);

            telemetry.addData("Step: ", step);

            telemetry.update();
        }
    }
}