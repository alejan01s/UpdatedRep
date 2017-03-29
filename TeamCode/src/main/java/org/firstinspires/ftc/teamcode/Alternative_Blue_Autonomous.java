package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by aleja on 3/26/2017.
 */

@Autonomous (name = "Blue Alternative Autonomous", group = "Sensors")
public class Alternative_Blue_Autonomous extends LinearOpMode {

    //MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    //VARIABLES FOR LAUNCHER
    public double EncoderClicks;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public DcMotor LauncherM;
    public Servo Reloader;
    public boolean turnCompleted = false;

    //SENSORS
    public ColorSensor colorSensor;
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

    boolean lookEnable = false;
    boolean recordRealVal = false;

    public boolean firstCollect = true;

    public boolean launcherCorrect = true;

    //REVOLUTION VARIABLES
    int NumberOfRevs1 = 400;
    int NumberOfRevs2 = -900;

    double step = 0;

    public void initializeRobot () {

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

    public void runOpMode () throws InterruptedException {

        initializeRobot();

        imuTest imu = new imuTest("imu", hardwareMap);
        sonarReader sonar = new sonarReader("SonarL", "SonarR", hardwareMap);
        colorSensorReader color = new colorSensorReader("ColorL", "ColorR", hardwareMap);

        while(!isStarted()){
            telemetry.addData("Status: ","Initialization Complete.");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){

            //GET VALUES FROM DRIVERS

            //IMU
            double[] angles = imu.getAngles();
            double yaw = angles[0];
            double x = yaw;

            if(x < 0){
                x = x + 360;
            }

            //SONAR
            double[] distances = sonar.getDistances();

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

                while (FR.getCurrentPosition() < NumberOfRevs1) {

                    FR.getCurrentPosition();

                    FL.setPower(0.6);
                    BL.setPower(0.6);
                    FR.setPower(0.6);
                    BR.setPower(0.6);

                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                step = step + 1;


            }

            if (step == 1) {

                while (yaw > -16 || yaw < -19) {

                    angles = imu.getAngles();
                    yaw = angles[0];

                    if (yaw < -16 && yaw > -19) {

                        break;

                    } else if (yaw > -16) {

                        FL.setPower(-0.4);
                        BL.setPower(-0.4);
                        FR.setPower(0.4);
                        BR.setPower(0.4);

                    } else if (yaw < -19) {

                        FL.setPower(0.4);
                        BL.setPower(0.4);
                        FR.setPower(-0.4);
                        BR.setPower(-0.4);

                    }

                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                NumberOfRevs2 = FR.getCurrentPosition() + 4750;

                step = step + 1;

            }

            if (step == 2) {
                if (FR.getCurrentPosition() < NumberOfRevs2 - 850) {

                    FL.setPower(1);
                    BL.setPower(1);
                    FR.setPower(1);
                    BR.setPower(1);

                } else if (FR.getCurrentPosition() < NumberOfRevs2) {

                    FL.setPower(0.35);
                    BL.setPower(0.35);
                    FR.setPower(0.35);
                    BR.setPower(0.35);

                } else if (FR.getCurrentPosition() > NumberOfRevs2) {

                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);

                    step = step + 1;

                }


            }

            if (step == 3) {

                while (yaw > -3 || yaw < -4) {

                    angles = imu.getAngles();
                    yaw = angles[0];

                    if (yaw < -3 && yaw > -4) {

                        break;

                    } else if (yaw > -3) {

                        FL.setPower(-0.2);
                        BL.setPower(-0.2);
                        FR.setPower(0.2);
                        BR.setPower(0.2);

                    } else if (yaw < -4) {

                        FL.setPower(0.2);
                        BL.setPower(0.2);
                        FR.setPower(-0.2);
                        BR.setPower(-0.2);

                    }

                }

                telemetry.update();

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                step = step + 1;

            }

            if (step == 4) {
                if(distanceRight > 40){
                    FR.setPower(-.15);
                    BR.setPower(.15);
                    FL.setPower(.15);
                    BL.setPower(-.15);

                }
                else{
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    step = step + 1;
                }
            }

            if (step == 5) {

                while (bottomOD.getRawLightDetected() < .04) {

                    bottomOD.getLightDetected();

                    if (bottomOD.getRawLightDetected() > 0.4) {

                        break;

                    }

                    FR.setPower(.4);
                    BR.setPower(.4);
                    FL.setPower(.4);
                    BL.setPower(.4);
                }

                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);

                NumberOfRevs1 = FR.getCurrentPosition() + 15;

                step = step + 1;

            }

            if (step == 6) {

                if (FR.getCurrentPosition() < NumberOfRevs1) {

                    FR.setPower(-0.2);
                    BR.setPower(-0.2);
                    FL.setPower(-0.2);
                    BL.setPower(-0.2);

                } else {

                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);

                    step = step + 1;

                }

            }

            if (step == 7) {
                NumberOfRevs4 = FL.getCurrentPosition() - 30;
                NumberOfRevs3 = FL.getCurrentPosition() - 240;
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

                    if (FL.getCurrentPosition() > NumberOfRevs4) {

                        BL.setPower(-.2);
                        BR.setPower(-.2);
                        FR.setPower(-.2);
                        FL.setPower(-.2);

                    } else {

                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        step = step + 1;
                    }

                } else if (OppPushSequence) {
                    if (FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.2);
                        BR.setPower(-.2);
                        FR.setPower(-.2);
                        FL.setPower(-.2);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        step = step + 1;
                    }
                }
            }

            if (step == 9) {

                NumberOfRevs1 = FR.getCurrentPosition() - 3000;
                if (OppPushSequence) {

                    NumberOfRevs1 = FR.getCurrentPosition() - 2500;

                }
                step = step + 1;

            }

            if (step == 10) {

                while (FR.getCurrentPosition() > NumberOfRevs1) {
                    FR.getCurrentPosition();

                    if (FR.getCurrentPosition() > NumberOfRevs1 - 800) {

                        FR.setPower(-1);
                        BR.setPower(-1);
                        FL.setPower(-1);
                        BL.setPower(-1);

                    } else if (FR.getCurrentPosition() > NumberOfRevs1) {

                        FR.setPower(-0.5);
                        BR.setPower(-0.5);
                        FL.setPower(-0.5);
                        BL.setPower(-0.5);

                    } else if (FR.getCurrentPosition() < NumberOfRevs1) {

                        break;

                    }
                }


                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);

                step = step + 1;

            }

            if(step == 11)
            {

                while (yaw > 0 || yaw < -1) {

                    angles = imu.getAngles();
                    yaw = angles[0];

                    if (yaw < 0 && yaw > -1) {

                        break;

                    } else if (yaw > 0) {

                        FL.setPower(-0.2);
                        BL.setPower(-0.2);
                        FR.setPower(0.2);
                        BR.setPower(0.2);

                    } else if (yaw < -1) {

                        FL.setPower(0.2);
                        BL.setPower(0.2);
                        FR.setPower(-0.2);
                        BR.setPower(-0.2);

                    }

                }

                telemetry.update();

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                step = step + 1;

            }

            if (step == 12) {
                step = step + 1;
            }

            if (step == 13) {
                if(distanceRight > 40){
                    FR.setPower(-.15);
                    BR.setPower(.15);
                    FL.setPower(.15);
                    BL.setPower(-.15);

                }
                else {
                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);
                    step = step + 1;
                }
            }

            if (step == 14) {

                while (bottomOD.getRawLightDetected() < .04) {

                    bottomOD.getLightDetected();

                    if (bottomOD.getRawLightDetected() > 0.4) {

                        break;

                    }

                    FR.setPower(.4);
                    BR.setPower(.4);
                    FL.setPower(.4);
                    BL.setPower(.4);
                }

                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);

                NumberOfRevs1 = FR.getCurrentPosition() + 15;

                step = step + 1;

            }

            if (step == 15) {

                if (FR.getCurrentPosition() < NumberOfRevs1) {

                    FR.setPower(-0.15);
                    BR.setPower(-0.15);
                    FL.setPower(-0.15);
                    BL.setPower(-0.15);

                } else {

                    FR.setPower(0);
                    BR.setPower(0);
                    FL.setPower(0);
                    BL.setPower(0);

                    step = step + 1;

                }

            }

            if (step == 16) {
                nearPush = false;
                OppPushSequence = false;

                NumberOfRevs4 = FL.getCurrentPosition() - 30;
                NumberOfRevs3 = FL.getCurrentPosition() - 300;
                step = step + 1;
            }

            if (step == 17) {
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

                    if (FL.getCurrentPosition() > NumberOfRevs4) {

                        BL.setPower(-.15);
                        BR.setPower(-.15);
                        FR.setPower(-.15);
                        FL.setPower(-.15);

                    } else {

                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        step = step + 1;

                    }

                } else if (OppPushSequence) {
                    if (FL.getCurrentPosition() > NumberOfRevs3) {
                        BL.setPower(-.15);
                        BR.setPower(-.15);
                        FR.setPower(-.15);
                        FL.setPower(-.15);
                    } else {
                        BL.setPower(0);
                        BR.setPower(0);
                        FR.setPower(0);
                        FL.setPower(0);
                        step = step + 1;

                    }
                }
            }


            if (step == 18)
            {


                if(OppPushSequence) {

                    while (yaw > -88 || yaw < -92) {

                        angles = imu.getAngles();
                        yaw = angles[0];

                        if (yaw < -88 && yaw > -92) {

                            break;

                        } else if (yaw > -88) {

                            FL.setPower(-0.4);
                            BL.setPower(-0.4);
                            FR.setPower(0.4);
                            BR.setPower(0.4);

                        } else if (yaw < -92) {

                            FL.setPower(0.4);
                            BL.setPower(0.4);
                            FR.setPower(-0.4);
                            BR.setPower(-0.4);

                        }

                    }

                    telemetry.update();

                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);

                    NumberOfRevs1 = FR.getCurrentPosition() - 50;

                    step = step + 1;
                }

                else
                {

                    while (yaw > -83 || yaw < -87) {

                        angles = imu.getAngles();
                        yaw = angles[0];

                        if (yaw < -83 && yaw > -87) {

                            break;

                        } else if (yaw > -83) {

                            FL.setPower(-0.4);
                            BL.setPower(-0.4);
                            FR.setPower(0.4);
                            BR.setPower(0.4);

                        } else if (yaw < -87) {

                            FL.setPower(0.4);
                            BL.setPower(0.4);
                            FR.setPower(-0.4);
                            BR.setPower(-0.4);

                        }

                    }

                    telemetry.update();

                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);

                    NumberOfRevs1 = FR.getCurrentPosition() - 50;

                    step = step + 1;

                }

            }

            if(step == 19)
            {
                step = step + 1;
            }

            if(step == 20)
            {


                while(FR.getCurrentPosition() > NumberOfRevs1) {

                    FR.getCurrentPosition();

                    if (FR.getCurrentPosition() > NumberOfRevs1) {
                        FL.setPower(-0.3);
                        BL.setPower(-0.3);
                        FR.setPower(-0.3);
                        BR.setPower(-0.3);

                    } else {

                        break;

                    }
                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                step = step + 1;

            }

            //LAUNCH BALLS
            if(step == 21){
                if(!fired) {
                    shoot1 = true;
                }
                if(fired) {
                    step = step + 1;
                }
            }
            if(step == 22){

                fired = false;

                if(!shoot) {
                    shoot = true;
                    step=step+1;
                }
            }

            if (step == 23 && fired == true)
            {

                while (yaw > -8 || yaw < -10) {

                    angles = imu.getAngles();
                    yaw = angles[0];

                    if (yaw < -8 && yaw > -10) {

                        break;

                    } else if (yaw > -8) {

                        FL.setPower(-0.4);
                        BL.setPower(-0.4);
                        FR.setPower(0.4);
                        BR.setPower(0.4);

                    } else if (yaw < -10) {

                        FL.setPower(0.4);
                        BL.setPower(0.4);
                        FR.setPower(-0.4);
                        BR.setPower(-0.4);

                    }

                }

                telemetry.update();

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                NumberOfRevs1 = FR.getCurrentPosition() - 2000;

                step = step + 1;

            }

            if(step == 24)
            {

                while(FR.getCurrentPosition() > NumberOfRevs1) {

                    FR.getCurrentPosition();

                    if (FR.getCurrentPosition() > NumberOfRevs1) {
                        FL.setPower(-0.8);
                        BL.setPower(-0.8);
                        FR.setPower(-0.8);
                        BR.setPower(-0.8);

                    } else {

                        break;

                    }
                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                step = step + 1;

            }

            //BUTTON PUSHER
            if (!buttonInit) {
                buttonPusher.setPosition(.5);
                if (push) {
                    buttonInit = true;
                }
            } else {
                buttonPusher.setPosition(.6);
                sleep(1800);
                buttonPusher.setPosition(.4);
                sleep(1800);
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
            if (!shoot && !shoot1) {
                if (LauncherM.getCurrentPosition() > (EncoderClicks - 2510)) {

                    LauncherM.setPower(-0.07);

                }

                if (LauncherM.getCurrentPosition() < (EncoderClicks - 2510)) {

                    LauncherM.setPower(0.07);

                }
            }


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
            idle();
        }

    }
}
