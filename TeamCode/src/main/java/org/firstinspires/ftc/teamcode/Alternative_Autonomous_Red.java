package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by trevo on 03/15/2017.
 */
@Disabled
@Autonomous(name = "Alternative Red", group = "Sensor")

public class Alternative_Autonomous_Red extends OpMode {

    /*

    I2C GLOBAL VARIABLES

     */
    //Defining Color Sensors
    I2cDevice ColorR;
    I2cDevice ColorL;
    I2cDeviceSynch ColorRReader;
    I2cDeviceSynch ColorLReader;

    //Defining Color Sensor Variables
    byte[] ColorRNumber;
    byte[] ColorLNumber;
    double CRNumber;
    double CLNumber;

    byte[] ColorRRed;
    byte[] ColorLRed;
    double CRRed;
    double CLRed;

    byte[] ColorRGreen;
    byte[] ColorLGreen;
    double CRGreen;
    double CLGreen;

    byte[] ColorRBlue;
    byte[] ColorLBlue;
    double CRBlue;
    double CLBlue;

    byte[] ColorRWhite;
    byte[] ColorLWhite;
    double CRWhite;
    double CLWhite;

    boolean ColorReadGamepadToggle;



    //Defining Sonar Sensors
    I2cDevice SonarR;
    I2cDevice SonarL;
    I2cDeviceSynch SonarRReader;
    I2cDeviceSynch SonarLReader;

    //Defining Sonar Sensor Variables
    byte[] RightDistanceTimeH;
    byte[] LeftDistanceTimeH;
    byte[] RightDistanceTimeL;
    byte[] LeftDistanceTimeL;
    double RightDistanceTime;
    double LeftDistanceTime;
    double RightDistanceTimeHDouble;
    double RightDistanceTimeLDouble;
    double LeftDistanceTimeHDouble;
    double LeftDistanceTimeLDouble;

    byte[] RightDistanceIN;
    byte[] LeftDistanceIN;
    double RightIN;
    double LeftIN;

    byte[] RightDistanceCM;
    byte[] LeftDistanceCM;
    double RightCM;
    double LeftCM;

    //Switches for Taking Distance Snapshots
    boolean CollectDistanceTime = false;
    boolean CollectDistanceIN = false;
    boolean CollectDistanceCM = false;

    //Defining Sleep Method Variables
    double Sleep = 0;
    double WakeUpTime;
    double SleepEnable = 0;

    /*
    ---------------------------------------------------------------------------------------------------
    AUTONOMOUS VARIABLES

     */
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

    imuTest imu;

    @Override
    public void init () {
        /*

        I2CC INIT

         */

        //Setting Up the Color Sensors

        //Mapping Color Sensors
        ColorR = hardwareMap.i2cDevice.get("ColorR");
        ColorL = hardwareMap.i2cDevice.get("ColorL");

        //Telling the Robot Which I2C Address to Talk To
        ColorRReader = new I2cDeviceSynchImpl(ColorR, I2cAddr.create8bit(0x3a), false);
        ColorLReader = new I2cDeviceSynchImpl(ColorL, I2cAddr.create8bit(0x3c), false);

        //Turning On Color Sensors
        ColorRReader.engage();
        ColorLReader.engage();



        //Setting Up the Sonar Sensors

        //Mapping Sonar Sensors
        SonarR = hardwareMap.i2cDevice.get("SonarR");
        SonarL = hardwareMap.i2cDevice.get("SonarL");

        //Telling the Robot Which I2C Address to Talk To
        SonarRReader = new I2cDeviceSynchImpl(SonarR, I2cAddr.create8bit(0xe0), false);
        SonarLReader = new I2cDeviceSynchImpl(SonarL, I2cAddr.create8bit(0xe2), false);

        //Turning on Sonar Sensors
        SonarRReader.engage();
        SonarLReader.engage();

        //Below are the range and gain settings for the sonar sensors

        //Right Sonar Sensor Configuration
        SonarRReader.write8(1, 3);//Gain
        SonarRReader.write8(2, 6);//Range

        //Left Sonar Sensor Configuration
        SonarLReader.write8(1, 3);//Gain
        SonarLReader.write8(2, 6);//Range



        //Putting the Robot to Sleep for 200ms to Ensure Settings are Written to Sonar Sensors
        Sleep = 200;

        if(SleepEnable == 0)
        {

            WakeUpTime = System.currentTimeMillis() + Sleep;

            while(System.currentTimeMillis() < WakeUpTime)
            {

            }

            Sleep = 0;

            SleepEnable = 1;

        }

        //Notifying Operator that the Initialization Routine has Finished
        telemetry.addLine("Initialization Complete");


        /*
        ---------------------------------------------------------------------
        AUTONOMOUS INIT

         */
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

        imu = new imuTest("imu", hardwareMap);
    }
    //SEQUENCE VARIABLE
    double step = 0;
    double ButtonPusherExtensionStep = -1;
    double ButtonPusherRetractionStep = -1;
    boolean StartPush = false;
    boolean PusherExtend = false;
    boolean PusherRetract = false;
    boolean EndPush = false;
    boolean Push1 = false;
    boolean Push1Go = false;
    boolean Push1End = false;
    boolean Push1Complete = false;
    boolean Push2 = false;
    boolean Push2Go = false;
    boolean Push2End = false;
    boolean Push2Complete = false;
    boolean Push3 = false;
    boolean Push3Go = false;
    boolean Push3End = false;
    boolean Push3Complete = false;
    boolean Push4 = false;
    boolean Push4Go = false;
    boolean Push4End = false;
    boolean Push4Complete = false;
    boolean Push5 = false;
    boolean Push5Go = false;
    boolean Push5End = false;
    boolean Push5Complete = false;
    boolean Push6 = false;
    boolean Push6Go = false;
    boolean Push6End = false;
    boolean Push6Complete = false;
    boolean Ignore = false;
    double PusherSleep = 0;

    //REVOLUTION VARIABLES
    int NumberOfRevs1 = 400;
    int NumberOfRevs2 = -900;

    //ANGLE VARIABLES
    double Angle1 = 190;
    double Angle2 = 280;

    //PRACTICE VALUES: .035, .05
    //USRA VALUES: .075, .08
    double beaconOneDistance = .053;
    double beaconTwoDistance = .056;

    boolean sleepOn = false;
    double timeToSleep;
    double timeToWake;

    double rightDisIN;
    double leftDisIN;
    double rightDisCM;
    double leftDisCM;
    double rightDisMS;
    double leftDisMS;

    boolean runCheck = false;

    @Override
    public void start () {
        //telemetry.addData("Status", "Initialization Complete");
        //telemetry.update();

        launcherCorrect = false;
        EncoderClicks = LauncherM.getCurrentPosition() + 2520;
    }

    @Override
    public void loop () {

            /*

            I2C LOOP

             */

        boolean StartRead = false;
        boolean LookForValue = false;
        boolean RecordValue = false;

        //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
        RightDistanceTimeH = SonarRReader.read(0x02, 1);
        RightDistanceTimeL = SonarRReader.read(0x03, 1);

        //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
        LeftDistanceTimeH = SonarLReader.read(0x02, 1);
        LeftDistanceTimeL = SonarLReader.read(0x03, 1);


        //Reading the Color Sensors

        //Obtaining the Red Value
        ColorRRed = ColorRReader.read(0x05, 1);
        ColorLRed = ColorLReader.read(0x05, 1);

        CRRed = ColorRRed[0] & 0xFF;
        CLRed = ColorLRed[0] & 0xFF;

        //Obtaining the Blue Value
        ColorRBlue = ColorRReader.read(0x07, 1);
        ColorLBlue = ColorLReader.read(0x07, 1);

        CRBlue = ColorRBlue[0] & 0xFF;
        CLBlue = ColorLBlue[0] & 0xFF;

//            //Reading the Sonar Sensors
//
//            //Reading Distance in MicroSeconds
//            if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
//            {
//
//                StartRead = true;
//
//
//                CollectDistanceTime = false;
//            }
//
//            if(StartRead == true) {
//                //Command the Sonars to Take a Snapshot
//                SonarRReader.write8(0, 82);
//                SonarLReader.write8(0, 82);
//
//                LookForValue = true;
//                StartRead = false;
//
//            }
//
//            if(LookForValue == true)
//            {
//
//                //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
//                RightDistanceTimeH = SonarRReader.read(0x02, 1);
//                RightDistanceTimeL = SonarRReader.read(0x03, 1);
//                RightDistanceTimeHDouble = RightDistanceTimeH[0] & 0xFF;
//                RightDistanceTimeLDouble = RightDistanceTimeL[0] & 0xFF;
//
//                //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
//                LeftDistanceTimeH = SonarLReader.read(0x02, 1);
//                LeftDistanceTimeL = SonarLReader.read(0x03, 1);
//                LeftDistanceTimeHDouble = LeftDistanceTimeH[0] & 0xFF;
//                LeftDistanceTimeLDouble = LeftDistanceTimeL[0] & 0xFF;
//
//            }
//
//            if(((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255)))
//            {
//
//                LookForValue = false;
//                RecordValue = true;
//
//            }
//
//            if(RecordValue == true) {
//
//                //Save Full Distance Values from Last Snapshot
//                RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
//                LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
//            }

            /*
            -------------------------------------------------------
            AUTONOMOUS LOOP

             */
        bottomOD.enableLed(true);
        frontOD.enableLed(true);

        if (firstCollect) {
            CollectDistanceTime = true;
            firstCollect = false;
        }

        //colorSensor.enableLed(false);
        //isRed = colorSensor.red() >= 1 && colorSensor.red() > colorSensor.blue() ? true : false;
        //isBlue = colorSensor.blue() >= 1 && colorSensor.blue() > colorSensor.red() ? true : false;

        isRed = CRRed > CRBlue && CRRed >= 1 ? true : false;
        isBlue = CRBlue > CRRed && CRBlue >= 1 ? true : false;

        telemetry.addData("Current Time: ", System.currentTimeMillis());
        telemetry.addData("PusherSleep: ", PusherSleep);
        telemetry.addData("Push1Go: ", Push1Go);
        telemetry.addData("Push1End: ", Push1End);
        telemetry.addData("Launcher", LauncherM.getCurrentPosition());
        telemetry.addData("Encoder Clicks: ", (EncoderClicks - 2520));

        telemetry.addData("FL: ", FL.getCurrentPosition());
        telemetry.addData("FR: ", FR.getCurrentPosition());
        telemetry.addData("BL: ", BL.getCurrentPosition());
        telemetry.addData("BR: ", BR.getCurrentPosition());

        telemetry.addData("colorOD: ", colorOD.getRawLightDetected());
        telemetry.addData("Red: ", CLRed);
        telemetry.addData("Blue: ", CLBlue);

        telemetry.addData("Left Distance Time: ", LeftDistanceTime);
        telemetry.addData("Bottom OD: ", bottomOD);

        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double pitch = angles[1];
        double roll = angles[2];
        double x = yaw;

        if (x < 0) {
            x = x + 360;
        }

        telemetry.addData(imu.getName(), imu.telemetrize());
        telemetry.addData("X: ", x);
        telemetry.update();

        NumberOfRevs1 = 300;

        //SEQUENCES
        BallG1.setPosition(0);
        BallG2.setPosition(1);
        buttonPusher.setPosition(.5);


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

            while (yaw < 16 || yaw > 19) {

                angles = imu.getAngles();
                yaw = angles[0];

                if (yaw > 16 && yaw < 19) {

                    break;

                } else if (yaw < 16) {

                    FL.setPower(0.4);
                    BL.setPower(0.4);
                    FR.setPower(-0.4);
                    BR.setPower(-0.4);

                } else if (yaw > 19) {

                    FL.setPower(-0.4);
                    BL.setPower(-0.4);
                    FR.setPower(0.4);
                    BR.setPower(0.4);

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

            if (!Push1Complete) {

                Push1 = true;

            }


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

            while (yaw > 6 || yaw < 5) {

                angles = imu.getAngles();
                yaw = angles[0];

                if (yaw < 5 && yaw > 6) {

                    break;

                } else if (yaw > 6) {

                    FL.setPower(-0.2);
                    BL.setPower(-0.2);
                    FR.setPower(0.2);
                    BR.setPower(0.2);

                } else if (yaw < 5) {

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

            while (RightDistanceTime > 1375 || RightDistanceTime == 0) {
                CollectDistanceTime = true;

                //Reading the Sonar Sensors

                //Reading Distance in MicroSeconds
                if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
                {

                    StartRead = true;


                    CollectDistanceTime = false;
                }

                if (StartRead == true) {
                    //Command the Sonars to Take a Snapshot
                    SonarRReader.write8(0, 82);
                    SonarLReader.write8(0, 82);

                    LookForValue = true;
                    StartRead = false;

                }

                if (LookForValue == true) {

                    //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
                    RightDistanceTimeH = SonarRReader.read(0x02, 1);
                    RightDistanceTimeL = SonarRReader.read(0x03, 1);
                    RightDistanceTimeHDouble = RightDistanceTimeH[0] & 0xFF;
                    RightDistanceTimeLDouble = RightDistanceTimeL[0] & 0xFF;

                    //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
                    LeftDistanceTimeH = SonarLReader.read(0x02, 1);
                    LeftDistanceTimeL = SonarLReader.read(0x03, 1);
                    LeftDistanceTimeHDouble = LeftDistanceTimeH[0] & 0xFF;
                    LeftDistanceTimeLDouble = LeftDistanceTimeL[0] & 0xFF;

                }

                if (((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255))) {

                    LookForValue = false;
                    RecordValue = true;

                }

                if (RecordValue == true) {

                    //Save Full Distance Values from Last Snapshot
                    RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
                    LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
                }

                if (RightDistanceTime < 1375 && RightDistanceTime != 0) {

                    break;

                }

                FR.setPower(.15);
                BR.setPower(-.15);
                FL.setPower(-.15);
                BL.setPower(.15);

            }

            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);

            CollectDistanceTime = false;

            step = step + 1;

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
            NumberOfRevs4 = FL.getCurrentPosition() - 16;
            NumberOfRevs3 = FL.getCurrentPosition() - 240;
            step = step + 1;
        }

        if (step == 8) {
            isRed = CRRed > CRBlue && CRRed >= 1 ? true : false;
            isBlue = CRBlue > CRRed && CRBlue >= 1 ? true : false;
            if (isRed && !OppPushSequence) {
                //push button
                nearPush = true;
            } else if (isBlue && !nearPush) {
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

//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
//
//                        if (!pushed) {
//                            push = true;
//                        } else {
//                            step = step + 1;
//
//
//                        }

                    if (!Push3Complete) {

                        Push2 = true;

                    } else {

                        step = step + 1;

                    }
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
//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
//                        if (!pushed) {
//                            push = true;
//                        }
//                        if (pushed) {
//                            step = step + 1;
//                        }
                        /*if(isBlue) {
                            if (!pushed) {
                                push = true;
                            }
                            if (pushed) {
                                step = step + 1;
                            }
                        }
                        else{
                            if(FL.getCurrentPosition() < NumberOfRevs3) {
                                BL.setPower(.1);
                                BR.setPower(.1);
                                FR.setPower(.1);
                                FL.setPower(.1);
                            }
                            else {
                                BL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                FL.setPower(0);
                                if (!pushed) {
                                    push = true;
                                }
                                if (pushed) {
                                    step = step + 1;
                                }
                            }
                        }*/

                    if (!Push3Complete) {

                        Push2 = true;

                    } else {

                        step = step + 1;

                    }

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

//                if (!Push5Complete) {
//
//                    Push5 = true;
//
//                }

                buttonPusher2.setPosition(0.6);


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

            buttonPusher2.setPosition(0.5);

            while (yaw < 0 || yaw > 1) {

                angles = imu.getAngles();
                yaw = angles[0];

                if (yaw > 0 && yaw < 1) {

                    break;

                } else if (yaw < 0) {

                    FL.setPower(0.2);
                    BL.setPower(0.2);
                    FR.setPower(-0.2);
                    BR.setPower(-0.2);

                } else if (yaw > 1) {

                    FL.setPower(-0.2);
                    BL.setPower(-0.2);
                    FR.setPower(0.2);
                    BR.setPower(0.2);

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

            RightDistanceTime = 10000;
            step = step + 1;

//            if (!Push6Complete) {
//
//                Push6 = true;
//
//            } else {
//
//                step = step + 1;
//
//            }

        }

        if (step == 13) {

            while (RightDistanceTime > 1050 || RightDistanceTime == 0) {
                CollectDistanceTime = true;

                //Reading the Sonar Sensors

                //Reading Distance in MicroSeconds
                if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
                {

                    StartRead = true;


                    CollectDistanceTime = false;
                }

                if (StartRead == true) {
                    //Command the Sonars to Take a Snapshot
                    SonarRReader.write8(0, 82);
                    SonarLReader.write8(0, 82);

                    LookForValue = true;
                    StartRead = false;

                }

                if (LookForValue == true) {

                    //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
                    RightDistanceTimeH = SonarRReader.read(0x02, 1);
                    RightDistanceTimeL = SonarRReader.read(0x03, 1);
                    RightDistanceTimeHDouble = RightDistanceTimeH[0] & 0xFF;
                    RightDistanceTimeLDouble = RightDistanceTimeL[0] & 0xFF;

                    //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
                    LeftDistanceTimeH = SonarLReader.read(0x02, 1);
                    LeftDistanceTimeL = SonarLReader.read(0x03, 1);
                    LeftDistanceTimeHDouble = LeftDistanceTimeH[0] & 0xFF;
                    LeftDistanceTimeLDouble = LeftDistanceTimeL[0] & 0xFF;

                }

                if (((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255))) {

                    LookForValue = false;
                    RecordValue = true;

                }

                if (RecordValue == true) {

                    //Save Full Distance Values from Last Snapshot
                    RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
                    LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
                }

                if (RightDistanceTime < 1075 && RightDistanceTime != 0) {

                    break;

                }

                FR.setPower(.15);
                BR.setPower(-.15);
                FL.setPower(-.15);
                BL.setPower(.15);

            }

            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);

            CollectDistanceTime = false;

            step = step + 1;

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

            Push3Complete = false;
            Push5Complete = false;
            Push6Complete = false;

            nearPush = false;
            OppPushSequence = false;

            NumberOfRevs4 = FL.getCurrentPosition() - 30;
            NumberOfRevs3 = FL.getCurrentPosition() - 300;
            step = step + 1;
        }

        if (step == 17) {
            isRed = CRRed > CRBlue && CRRed >= 1 ? true : false;
            isBlue = CRBlue > CRRed && CRBlue >= 1 ? true : false;
            if (isRed && !OppPushSequence) {
                //push button
                nearPush = true;
            } else if (isBlue && !nearPush) {
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

//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
//
//                        if (!pushed) {
//                            push = true;
//                        } else {
//                            step = step + 1;
//
//
//                        }

                    if (!Push3Complete) {

                        Push2 = true;

                    } else {

                        step = step + 1;

                    }
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
//                        sleepOn = true;
//                        timeToSleep = 5;
//
//                        if (sleepOn) {
//
//                            timeToWake = System.currentTimeMillis() + timeToSleep;
//
//                            while (System.currentTimeMillis() < timeToWake) {
//
//                            }
//
//                            timeToSleep = 0;
//
//                            sleepOn = false;
//
//                        }
//                        if (!pushed) {
//                            push = true;
//                        }
//                        if (pushed) {
//                            step = step + 1;
//                        }
                        /*if(isBlue) {
                            if (!pushed) {
                                push = true;
                            }
                            if (pushed) {
                                step = step + 1;
                            }
                        }
                        else{
                            if(FL.getCurrentPosition() < NumberOfRevs3) {
                                BL.setPower(.1);
                                BR.setPower(.1);
                                FR.setPower(.1);
                                FL.setPower(.1);
                            }
                            else {
                                BL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                FL.setPower(0);
                                if (!pushed) {
                                    push = true;
                                }
                                if (pushed) {
                                    step = step + 1;
                                }
                            }
                        }*/

                    if (!Push3Complete) {

                        Push2 = true;

                    } else {

                        step = step + 1;

                    }

                }
            }
        }

//        if(step == 17.5)
//        {
//
//            if(!Push6Complete)
//            {
//
//                Push6 = true;
//
//            }
//
//            else
//            {
//
//                step = step + 0.5;
//
//            }
//
//        }

        if (step == 18)
        {

            buttonPusher2.setPosition(0.6);

            if(OppPushSequence) {

                while (yaw < 90 || yaw > 94) {

                    angles = imu.getAngles();
                    yaw = angles[0];

                    if (yaw < 94 && yaw > 90) {

                        break;

                    } else if (yaw > 94) {

                        FL.setPower(-0.4);
                        BL.setPower(-0.4);
                        FR.setPower(0.4);
                        BR.setPower(0.4);

                    } else if (yaw < 90) {

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

                while (yaw > 91 || yaw < 87) {

                    angles = imu.getAngles();
                    yaw = angles[0];

                    if (yaw < 91 && yaw > 87) {

                        break;

                    } else if (yaw > 91) {

                        FL.setPower(-0.4);
                        BL.setPower(-0.4);
                        FR.setPower(0.4);
                        BR.setPower(0.4);

                    } else if (yaw < 87) {

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

                NumberOfRevs1 = FR.getCurrentPosition() + 75;

                step = step + 1;

            }

        }

        if(step == 19)
        {

            Push6Complete = false;

            step = step + 1;

        }

        if(step == 20)
        {


//            while(FR.getCurrentPosition() < NumberOfRevs1) {
//
//                FR.getCurrentPosition();
//
//                if (FR.getCurrentPosition() < NumberOfRevs1) {
//                    FL.setPower(0.3);
//                    BL.setPower(0.3);
//                    FR.setPower(0.3);
//                    BR.setPower(0.3);
//
//                } else {
//
//                    break;
//
//                }
//            }
//
//            FL.setPower(0);
//            BL.setPower(0);
//            FR.setPower(0);
//            BR.setPower(0);

            step = step + 1;

        }

        //LAUNCH BALLS
        if(step == 21){

            buttonPusher2.setPosition(0.5);
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

            while (yaw > 16 || yaw < 12) {

                angles = imu.getAngles();
                yaw = angles[0];

                if (yaw < 16 && yaw > 12) {

                    break;

                } else if (yaw > 16) {

                    FL.setPower(-0.4);
                    BL.setPower(-0.4);
                    FR.setPower(0.4);
                    BR.setPower(0.4);

                } else if (yaw < 12) {

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

            buttonPusher2.setPosition(0.6);

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

        if(step == 25)
        {

            buttonPusher2.setPosition(0.5);

        }



            /*

            Button Pusher

             */

        if(Push1 && Ignore == false)
        {

            Push1Complete = false;
            PusherSleep = System.currentTimeMillis() + 850;
            buttonPusher2.setPosition(0.4);
            Push1Go = true;
            Push1 = false;

        }

        if(Push1Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {

                if(System.currentTimeMillis() > PusherSleep)
                {

                    break;

                }

            }

            buttonPusher2.setPosition(0.5);
            Push1End = true;
            Push1Go = false;
        }

        if(Push1End && Ignore == false)
        {


            Push1Complete = true;
            Push1End = false;

        }



        if(Push2 && Ignore == false)
        {

            Push2Complete = false;
            PusherSleep = System.currentTimeMillis() + 2400;
            buttonPusher2.setPosition(0.4);
            Push2Go = true;
            Push2 = false;

        }

        if(Push2Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {

                if(System.currentTimeMillis() > PusherSleep)
                {

                    break;

                }

            }

            buttonPusher2.setPosition(0.5);
            Push2End = true;
            Push2Go = false;
        }

        if(Push2End && Ignore == false)
        {


            Push2Complete = true;
            Push2End = false;

        }

        if(Push2Complete && Ignore == false)
        {

            Push3 = true;
            Push2Complete = false;

        }



        if(Push3 && Ignore == false)
        {

            Push3Complete = false;
            PusherSleep = System.currentTimeMillis() + 1000;
            buttonPusher2.setPosition(0.6);
            Push3Go = true;
            Push3 = false;

        }

        if(Push3Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {

                if(System.currentTimeMillis() > PusherSleep)
                {

                    break;

                }

            }

            buttonPusher2.setPosition(0.5);
            Push3End = true;
            Push3Go = false;
        }

        if(Push3End && Ignore == false)
        {


            Push3Complete = true;
            Push3End = false;

        }





        if(Push4 && Ignore == false)
        {

            Push4Complete = false;
            PusherSleep = System.currentTimeMillis() + 2000;
            buttonPusher2.setPosition(0.4);
            Push4Go = true;
            Push4 = false;

        }

        if(Push4Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {

                if(System.currentTimeMillis() > PusherSleep)
                {

                    break;

                }

            }

            buttonPusher2.setPosition(0.5);
            Push4End = true;
            Push4Go = false;
        }

        if(Push4End && Ignore == false)
        {


            Push4Complete = true;
            Push4End = false;

        }

//            if(Push4Complete)
//            {
//
//                Push5 = true;
//                Push4Complete = false;
//
//            }



        if(Push5 && Ignore == false)
        {

            Push5Complete = false;
            PusherSleep = System.currentTimeMillis() + 350;
            buttonPusher2.setPosition(0.6);
            Push5Go = true;
            Push5 = false;

        }

        if(Push5Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {

                if(System.currentTimeMillis() > PusherSleep)
                {

                    break;

                }

            }

            buttonPusher2.setPosition(0.5);
            Push5End = true;
            Push5Go = false;
        }

        if(Push5End && Ignore == false)
        {


            Push5Complete = true;
            Push5End = false;

        }



        if(Push6 && Ignore == false)
        {

            Push6Complete = false;
            PusherSleep = System.currentTimeMillis() + 750;
            buttonPusher2.setPosition(0.6);
            Push6Go = true;
            Push6 = false;

        }

        if(Push6Go && Ignore == false)
        {

            while(System.currentTimeMillis() < PusherSleep)
            {

                if(System.currentTimeMillis() > PusherSleep)
                {

                    break;

                }

            }

            buttonPusher2.setPosition(0.5);
            Push6End = true;
            Push6Go = false;
        }

        if(Push6End && Ignore == false)
        {


            Push6Complete = true;
            Push6End = false;

        }




            /*

            SHOOTING SYSTEM

             */

        if(shoot) {

            fired = false;

            if(LauncherM.getCurrentPosition() <= 1000 + (EncoderClicks - 2520))
            {

                Reloader.setPosition(0.7);
                LauncherM.setPower(0.75);

            }

            else if(LauncherM.getCurrentPosition() <= 1200 + (EncoderClicks - 2520))
            {

                Reloader.setPosition(0.1);

            }

            else if(LauncherM.getCurrentPosition() <= 2520 + (EncoderClicks - 2520))
            {

                Reloader.setPosition(0.1);
                LauncherM.setPower(0.85);

            }

            else if (LauncherM.getCurrentPosition() > 2520 + (EncoderClicks - 2520) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                LauncherM.setPower(.1);
            } else {
                LauncherM.setPower(0);
                shoot = false;
                EncoderClicks = EncoderClicks + 2520;

                fired = true;
            }
        }

        if(shoot1){
            if (LauncherM.getCurrentPosition() <= EncoderClicks - 500) {
                LauncherM.setPower(0.85);
            }
            else{
                LauncherM.setPower(0);
                fired = true;
                shoot1 = false;
                EncoderClicks = EncoderClicks + 2520;
            }
        }
//
//                if(launcherCorrect) {
//
////                    if(LauncherM.getCurrentPosition() > (EncoderClicks - 2400))
////                    {
////
////                        LauncherM.setPower(-0.3);
////
////                    }
//
//                     if (LauncherM.getCurrentPosition() > (EncoderClicks - 2500)) {
//
//                        LauncherM.setPower(-0.03);
//
//                    }
//
//                    else if (LauncherM.getCurrentPosition() < (EncoderClicks - 2540)) {
//
//                        LauncherM.setPower(0.03);
//
//                    }
//
//                    else if ((LauncherM.getCurrentPosition() < (EncoderClicks - 2500)) && (LauncherM.getCurrentPosition() > (EncoderClicks - 2540)))
//                    {
//
//                        LauncherM.setPower(0);
//
//                    }
//                }
//
//
//                if((!launcherCorrect) && (step > 3))
//                {
//
//                    LauncherM.setPower(0);
//
//                }

//            else if(step > 11){
//
//                LauncherM.setPower(0);
//
//            }
    }

    @Override
    public void stop () {

        buttonPusher.setPosition(0.5);

    }
}

