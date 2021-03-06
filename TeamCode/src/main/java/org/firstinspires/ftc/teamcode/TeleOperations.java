package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

/**
 * Created by aleja on 10/14/2016.
 */

@TeleOp(name="Main TeleOp", group="Linear Opmode")
public class TeleOperations extends LinearOpMode {
    Timer timer = new Timer();

    public boolean downenabled;
    public boolean upenabled;
    public boolean upstopped;
    public boolean downstopped;

    //DRIVE-TRAIN MOTORS
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    //ROLLER
    public DcMotor Roller;

    //LAUNCHER VARIABLES
    public double EncoderClicks;
    public boolean shoot = false;
    public boolean pause = false;
    public boolean resume = false;
    public boolean fire = false;

    //ROLLER VARIABLES
    public double rollerState = 0;

    public DcMotor LiftL;
    public DcMotor LiftR;
    public double encodersLift;
    public boolean tensionLift;

    public Servo BallG1;
    public Servo BallG2;

    public boolean waitForDetoggle = false;
    public boolean manualOverrideBallG1;
    public boolean manualOverrideBallG2;

    public double posBallG1 = 0;
    public double posBallG2 = 0;

    public double overridePosL = 0;
    public double overridePosR = 0;

    public boolean runUp = false;
    public boolean runDown = false;
    public boolean cancelOverride = false;

    public long buttonHold;
    public boolean moveButton;

    //LAUNCHER MECHANISM
    public DcMotor LauncherM;
    public Servo Reloader;

    public Servo buttonPusher;
    public Servo buttonPusher2;
    public boolean buttonPress;
    public boolean buttonInit;

    public boolean overrideLift;

    public boolean runReload;
    public boolean runReloaderDown;

    public boolean setEncoder = true;
    public boolean armMoving = false;

    public boolean overrideLauncher = false;

    public void initializeRobot() throws InterruptedException {

        //CONFIGURATION
        BL = hardwareMap.dcMotor.get("Bl");
        BR = hardwareMap.dcMotor.get("Br");
        FL = hardwareMap.dcMotor.get("Fl");
        FR = hardwareMap.dcMotor.get("Fr");

        Roller = hardwareMap.dcMotor.get("Roller");

        LauncherM = hardwareMap.dcMotor.get("Launcher");
        Reloader = hardwareMap.servo.get("Reloader");

        LiftL = hardwareMap.dcMotor.get("LiftL");
        LiftR = hardwareMap.dcMotor.get("LiftR");

        BallG1 = hardwareMap.servo.get("BallG2");
        BallG2 = hardwareMap.servo.get("BallG1");

        //RUN USING ENCODERS + RESET THEM ON START + REVERSE
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

        LauncherM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        buttonPusher = hardwareMap.servo.get("buttonPusher");
        buttonPusher2 = hardwareMap.servo.get("buttonPusher2");

        //LiftL.setDirection(DcMotor.Direction.REVERSE);

        LiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //BallG2.setDirection(Servo.Direction.REVERSE);

        //super.initializeRobot();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initializeRobot();

        //VARIABLES FOR TOGGLING
        boolean SwitchState = false;
        boolean MoveLiftUp = false;
        boolean MoveLiftDown = false;

        boolean SwitchS = false;
        boolean MoveBGsBack = false;
        boolean MoveBGsFor = false;

        boolean buttonPress = false;
        boolean buttonInit = false;

        //ROLLER POWER
        double Roll = 0;

        boolean MoveLiftUpL = false;
        boolean MoveLiftUpR = false;
        boolean MoveLiftDownL = false;
        boolean MoveLiftDownR = false;
        boolean MoveLiftUpPos2L = false;
        boolean MoveLiftUpPos2R = false;
        boolean MoveLiftUpPos2 = false;
        boolean MoveLiftUpPos3 = false;
        boolean MoveLiftUpPos3L = false;
        boolean MoveLiftUpPos3R = false;
        boolean MoveLiftDownPos3 = false;
        boolean MoveLiftDownPos3L = false;
        boolean MoveLiftDownPos3R = false;
        boolean manualOverrideLiftUp;
        boolean manualOverrideLiftDown;
        boolean lRanUp = false;
        boolean rRanUp = false;
        boolean lRanDown = false;
        boolean rRanDown = false;
        tensionLift = false;

        while(setEncoder == true) {
            if (LauncherM.getCurrentPosition() < 3000 || LauncherM.getCurrentPosition() > 7000) {

                EncoderClicks = LauncherM.getCurrentPosition() + 2520;

                setEncoder = false;

            } else {

                EncoderClicks = 7560;

                setEncoder = false;

            }

        }

        waitForStart();
        while(opModeIsActive()){

            /*

            TELEMETRY DATA

             */

            telemetry.addData("Encoder Clicks: ", LauncherM.getCurrentPosition());

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());

            telemetry.addData("LiftL: ", LiftL.getCurrentPosition());
            telemetry.addData("LiftR: ", LiftR.getCurrentPosition());

            telemetry.addData("BallG1 Encoder: ", BallG1.getPosition());
            telemetry.addData("BallG2 Encoder: ", BallG2.getPosition());

            telemetry.addData("Roller Status: ", rollerState);

            telemetry.update();

            buttonPusher2.setPosition(.5);

            /*

            CODE FOR LAUNCHING MECHANISM

            */

            //360 DEGREES = 2745 CLICKS; 30 DEGREES HOLD FOR RELOAD; 130 DEGREES BALL FIRES
            /*
            if(gamepad1.right_trigger > .75){
                shoot = true;
            }

            if(shoot) {
                if(!resume) {
                    if (LauncherM.getCurrentPosition() <= 400 + (EncoderClicks - 2520)) {
                        LauncherM.setPower(1);
                    } else if (LauncherM.getCurrentPosition() <= 550 + (EncoderClicks - 2520)) {
                        LauncherM.setPower(1);
                    }
                    else{
                        pause = true;
                    }
                    if (pause) {
                        LauncherM.setPower(0.15);
                        Reloader.setPosition(.65);
                        resume = true;
                        pause = false;
                    }
                }
                if(resume) {
                    if (LauncherM.getCurrentPosition() > 550 + (EncoderClicks-2520) && LauncherM.getCurrentPosition() <= 1200 + (EncoderClicks - 2520)) {
                        LauncherM.setPower(.15);
                    }
                    else if (LauncherM.getCurrentPosition() > 1200 + (EncoderClicks-2520) && LauncherM.getCurrentPosition() <= 2250 + (EncoderClicks - 2520)) {
                        LauncherM.setPower(1);
                        Reloader.setPosition(0.1);
                    } else if (LauncherM.getCurrentPosition() > 2250 + (EncoderClicks - 2520) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                        LauncherM.setPower(.1);
                    } else {
                        LauncherM.setPower(0);
                        shoot = false;
                        resume = false;
                        EncoderClicks = EncoderClicks + 2520;
                    }
                }
            }
            */

            /*

            NEW SHOOTING SEQUENCE

            */

            if(gamepad1.right_trigger > .75){
                shoot = true;
            }

            if(shoot) {

                if(LauncherM.getCurrentPosition() <= 1000 + (EncoderClicks - 2520))
                {

                    Reloader.setPosition(0.43);
                    LauncherM.setPower(0.45);

                }

                else if(LauncherM.getCurrentPosition() <= 1200 + (EncoderClicks - 2520))
                {

                    Reloader.setPosition(0.003);

                }

                else if(LauncherM.getCurrentPosition() <= 2520 + (EncoderClicks - 2520))
                {

                    Reloader.setPosition(0.003);
                    LauncherM.setPower(0.85);

                }

                else if (LauncherM.getCurrentPosition() > 2520 + (EncoderClicks - 2520) && LauncherM.getCurrentPosition() <= EncoderClicks) {
                    LauncherM.setPower(.1);
                } else {
                    LauncherM.setPower(0);
                    shoot = false;
                    EncoderClicks = EncoderClicks + 2520;
                }

//                if(LauncherM.getCurrentPosition() <=325 + (EncoderClicks - 2520)) {
//                    Reloader.setPosition(.65);
//                    LauncherM.setPower(.42);
//                }
//                else if (LauncherM.getCurrentPosition() <= 500 + (EncoderClicks - 2520)) {
//                    LauncherM.setPower(1);
//                }
//                else if (LauncherM.getCurrentPosition() <= 2250 + (EncoderClicks - 2520)) {
//                    Reloader.setPosition(.1);
//                    LauncherM.setPower(1);
//                }
//                else if (LauncherM.getCurrentPosition() > 2250 + (EncoderClicks - 2520) && LauncherM.getCurrentPosition() <= EncoderClicks) {
//                    LauncherM.setPower(.1);
//                } else {
//                    LauncherM.setPower(0);
//                    shoot = false;
//                    EncoderClicks = EncoderClicks + 2520;
//                }
            }

            if(!shoot && !fire)
            {

                if(LauncherM.getCurrentPosition() > (EncoderClicks - 2520))
                {

                    LauncherM.setPower(-0.07);

                }

                if(LauncherM.getCurrentPosition() < (EncoderClicks - 2520))
                {

                    LauncherM.setPower(0.07);

                }

            }

            /*

            CODE FOR FIRE SEQUENCE ONLY

             */

            if(gamepad1.right_bumper){
                fire = true;
            }

            if(fire){
                if(LauncherM.getCurrentPosition() <= EncoderClicks){
                    LauncherM.setPower(1);
                }
                else{
                    LauncherM.setPower(0);
                    fire = false;
                    EncoderClicks = EncoderClicks + 2520;
                }
            }

            /*

            CODE FOR MANUALLY RESETTING LAUNCHER ENCODERS

             */

            if(gamepad1.a && gamepad1.b && gamepad1.y && gamepad1.x){
                LauncherM.setMode(DcMotor.RunMode.RESET_ENCODERS);
            }

            /*

            MECHANUM WHEEL CODE

            */


            //FINAL WORKING MECHANUM WHEEL CODE

            double x;
            double y;
            double x2;

            final double slowMode = 6;
            final double joystickThreshold = 10;

            if (Math.abs(100 * gamepad1.left_stick_x) > joystickThreshold) {
                y = gamepad1.left_stick_x;
            } else {
                y = 0;
            }
            if (Math.abs(100 * gamepad1.left_stick_y) > joystickThreshold) {
                x = gamepad1.left_stick_y;
            } else {
                x = 0;
            }
            if (Math.abs(100 * gamepad1.right_stick_x) > joystickThreshold) {
                x2 = gamepad1.right_stick_x;
            } else {
                x2 = 0;
            }
            if(!armMoving) {
                if (gamepad1.left_bumper) {
                    FR.setPower((y + x2 + x) / slowMode);
                    BR.setPower((-y + x2 + x) / slowMode);
                    FL.setPower((-y - x2 + x) / slowMode);
                    BL.setPower((y - x2 + x) / slowMode);
                } else {
                    FR.setPower(y + x2 + x);
                    BR.setPower(-y + x2 + x);
                    FL.setPower(-y - x2 + x);
                    BL.setPower(y - x2 + x);
                }
            }
            else{
                FR.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                BL.setPower(0);
            }
            /*

            CODE FOR ROLLERS

            */

            if(gamepad2.right_trigger > 0.1f && gamepad2.left_trigger < 0.1f){
                Roll = gamepad2.right_trigger;
            }
            else if(gamepad2.left_trigger > 0.1f && gamepad2.right_trigger < 0.1f){
                Roll = -gamepad2.left_trigger;
            }
            else {
                Roll = 0;
            }

            Roller.setPower(Roll);

            /*

            CODE FOR LIFT

            */

            //BUTTON X IS USED TO TOGGLE LIFT UP AND DOWN
            //MANUAL OVERRIDE IS MAPPED TO TRIGGERS

            //boolean CurrentState = gamepad2.x;
            boolean BallGStowed = BallG1.getPosition() > 0.015 && BallG2.getPosition() < .985 || gamepad2.dpad_down == true ? false : true;
            overrideLift = gamepad2.guide ? true : false;

            telemetry.addData("BallGStowed: ", BallGStowed);

            if(!BallGStowed) {
                if (!overrideLift) {
                    if (upenabled == true) {
                        if (LiftR.getCurrentPosition() < 8175) {
                            if (gamepad2.dpad_up == true) {
                                LiftR.setPower(1);
                                LiftL.setPower(-1);
                            }
                        }
                        if (LiftR.getCurrentPosition() >= 8175) {
                            upenabled = false;
                        }
                    }

                    if (downenabled == true) {
                        if (LiftR.getCurrentPosition() > 1000) {
                            if (gamepad2.dpad_down == true) {
                                LiftR.setPower(-0.75);
                                LiftL.setPower(0.75);
                            }
                        }

                        if (LiftR.getCurrentPosition() > 0 && LiftR.getCurrentPosition() <= 1000) {
                            if (gamepad2.dpad_down == true) {
                                LiftR.setPower(-0.3);
                                LiftL.setPower(0.3);
                            }
                        }

                        if (LiftR.getCurrentPosition() <= 0) {
                            downenabled = false;
                        }
                    }

                    if (upenabled == false) {
                        if (upstopped == false) {
                            LiftR.setPower(0);
                            LiftL.setPower(0);
                            upstopped = true;
                        }
                    }
                    if (downenabled == false) {
                        if (downstopped == false) {
                            LiftR.setPower(0);
                            LiftL.setPower(0);
                            downstopped = true;
                        }
                    }

                    if (LiftR.getCurrentPosition() < 8175) {
                        upenabled = true;
                        upstopped = false;
                    }

                    if (LiftR.getCurrentPosition() > 0) {
                        downenabled = true;
                        downstopped = false;
                    }

                    if (gamepad2.dpad_down == false && gamepad2.dpad_up == false) {
                        LiftR.setPower(0);
                        LiftL.setPower(0);
                    }
                }
                else{
                    if (upenabled == true) {
                        if (LiftR.getCurrentPosition() < 9000) {
                            if (gamepad2.dpad_up == true) {
                                LiftR.setPower(1);
                                LiftL.setPower(-1);
                            }
                        }
                        if (LiftR.getCurrentPosition() >= 9000) {
                            upenabled = false;
                        }
                    }

                    if (downenabled == true) {
                        if (LiftR.getCurrentPosition() > 1000) {
                            if (gamepad2.dpad_down == true) {
                                LiftR.setPower(-0.75);
                                LiftL.setPower(0.75);
                            }
                        }

                        if (LiftR.getCurrentPosition() > 0 && LiftR.getCurrentPosition() <= 1000) {
                            if (gamepad2.dpad_down == true) {
                                LiftR.setPower(-0.3);
                                LiftL.setPower(0.3);
                            }
                        }

                        if (LiftR.getCurrentPosition() <= 0) {
                            downenabled = false;
                        }
                    }

                    if (upenabled == false) {
                        if (upstopped == false) {
                            LiftR.setPower(0);
                            LiftL.setPower(0);
                            upstopped = true;
                        }
                    }
                    if (downenabled == false) {
                        if (downstopped == false) {
                            LiftR.setPower(0);
                            LiftL.setPower(0);
                            downstopped = true;
                        }
                    }

                    if (LiftR.getCurrentPosition() < 9000) {
                        upenabled = true;
                        upstopped = false;
                    }

                    if (LiftR.getCurrentPosition() > 0) {
                        downenabled = true;
                        downstopped = false;
                    }

                    if (gamepad2.dpad_down == false && gamepad2.dpad_up == false) {
                        LiftR.setPower(0);
                        LiftL.setPower(0);
                    }
                }
            }
            /*
            manualOverrideLiftUp = gamepad2.dpad_up ? true : false;
            manualOverrideLiftDown = gamepad2.dpad_down ? true : false;
            if(!cancelOverride) {
                if (manualOverrideLiftUp) {
                    overridePosL = LiftL.getCurrentPosition() - 5;
                    overridePosR = LiftR.getCurrentPosition() + 5;
                    lRanUp = false;
                    rRanUp = false;
                    runUp = true;
                } else {
                    overridePosR = LiftR.getCurrentPosition();
                    overridePosL = LiftL.getCurrentPosition();
                }
                if (manualOverrideLiftDown) {
                    overridePosL = LiftL.getCurrentPosition() + 1;
                    overridePosR = LiftR.getCurrentPosition() - 1;
                    lRanDown = false;
                    rRanDown = false;
                    runDown = true;
                } else {
                    overridePosL = LiftL.getCurrentPosition();
                    overridePosR = LiftR.getCurrentPosition();
                }

                if (runUp) {
                    if (LiftL.getCurrentPosition() > overridePosL) {
                        LiftL.setPower(-.65);
                    } else {
                        LiftL.setPower(0);
                        lRanUp = true;
                    }
                    if (LiftR.getCurrentPosition() < overridePosR) {
                        LiftR.setPower(.65);
                    } else {
                        LiftR.setPower(0);
                        rRanUp = true;
                    }
                    if (lRanUp && rRanUp) {
                        runUp = false;
                    }
                } else if (runDown) {
                    if (LiftL.getCurrentPosition() < overridePosL) {
                        LiftL.setPower(.25);
                    } else {
                        LiftL.setPower(0);
                        lRanDown = false;
                    }
                    if (LiftR.getCurrentPosition() > overridePosR) {
                        LiftR.setPower(-.25);
                    } else {
                        LiftR.setPower(0);
                        rRanDown = true;
                    }
                    if (lRanDown && rRanDown) {
                        runDown = false;
                    }
                }
            }
            boolean CurrentState = gamepad1.y;

            if (CurrentState == true && SwitchState != CurrentState && MoveLiftDownPos3 == false) {
                SwitchState = CurrentState;
                MoveLiftUpPos3 = true;
                MoveLiftUpPos3L = true;
                MoveLiftUpPos3R = true;
                tensionLift = true;

                Thread.sleep(5);
            } else if (CurrentState == true && SwitchState == CurrentState && MoveLiftUpPos3 == false) {
                SwitchState = false;
                MoveLiftDownPos3 = true;
                MoveLiftDownPos3L = true;
                MoveLiftDownPos3R = true;
                tensionLift = true;

                Thread.sleep(5);
            }
            if(MoveLiftUpPos3){
                if(tensionLift){
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1500);
                    tensionLift = false;
                }
                if(!tensionLift){
                    if (LiftL.getCurrentPosition() > -400) {
                        LiftL.setPower(-1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftUpPos3L = false;
                    }
                    if (LiftR.getCurrentPosition() < 400) {
                        LiftR.setPower(1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftUpPos3R = false;
                    }
                    if(!MoveLiftUpPos3L && !MoveLiftUpPos3R){
                        cancelOverride = false;
                        MoveLiftUpPos3 =false;
                    }
                }
            }
            if(MoveLiftDownPos3){
                if(tensionLift){
                    /*
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1500);

                    tensionLift = false;
                }
                if(!tensionLift){
                    if (LiftL.getCurrentPosition() < 0) {
                        LiftL.setPower(1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftDownPos3L = false;
                    }
                    if (LiftR.getCurrentPosition() > 0) {
                        LiftR.setPower(-1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftDownPos3R = false;
                    }
                    if(!MoveLiftDownPos3L && !MoveLiftDownPos3R){
                        cancelOverride = false;
                        MoveLiftDownPos3 = false;
                    }
                }
            }
            */
            //BALL GRABBERS MUST NOT BE STOWED TO RUN LIFT AS A SAFETY MEASURE
            if(!BallGStowed) {

                /*
                if (CurrentState == true && SwitchState != CurrentState && MoveLiftDown == false) {
                    SwitchState = CurrentState;
                    MoveLiftUp = true;
                    MoveLiftUpL = true;
                    MoveLiftUpR = true;
                    tensionLift = true;
                    encodersLift = LiftL.getCurrentPosition() - 8000;

                    Thread.sleep(5);
                } else if (CurrentState == true && SwitchState == CurrentState && MoveLiftUp == false) {
                    SwitchState = false;
                    MoveLiftDown = true;
                    MoveLiftDownL = true;
                    MoveLiftDownR = true;
                    tensionLift = true;

                    Thread.sleep(5);
                }
                */

                if(gamepad2.x){
                    MoveLiftUp = true;
                    MoveLiftUpL = true;
                    MoveLiftUpR = true;
                    cancelOverride = true;
                    //encodersLift = LiftL.getCurrentPosition() - 8000;
                }

                if(gamepad2.b){
                    MoveLiftDown = true;
                    MoveLiftDownL = true;
                    MoveLiftDownR = true;
                    cancelOverride = true;
                    tensionLift = true;
                }

                if(gamepad2.a){
                    MoveLiftUpPos2 = true;
                    MoveLiftUpPos2L = true;
                    MoveLiftUpPos2R = true;
                    cancelOverride = true;
                    tensionLift = true;
                }
                if(gamepad2.y){
                    MoveLiftUpPos3 = true;
                    MoveLiftUpPos3L = true;
                    MoveLiftUpPos3R = true;
                    cancelOverride = true;
                    tensionLift = true;
                }
            }

            //RAISE LIFT
            if(MoveLiftUp){
                if(tensionLift) {
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1500);
                    tensionLift = false;
                }
                if(!tensionLift) {
                    if (LiftL.getCurrentPosition() > -8175) {
                        LiftL.setPower(-1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftUpL = false;
                    }
                    if (LiftR.getCurrentPosition() < 8175) {
                        LiftR.setPower(1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftUpR = false;
                    }
                    if(!MoveLiftUpL && !MoveLiftUpR){
                        cancelOverride = false;
                        MoveLiftUp =false;
                    }
                }
            }
            //LOWER LIFT
            else if(MoveLiftDown){
                if(tensionLift) {
                    if (LiftL.getCurrentPosition() < 0) {
                        LiftL.setPower(1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftDownL = false;
                    }
                    if (LiftR.getCurrentPosition() > 0) {
                        LiftR.setPower(-1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftDownR = false;
                    }
                    if(!MoveLiftDownL && !MoveLiftDownR){
                        tensionLift = false;
                    }
                }
                if(!tensionLift){
                    //Thread.sleep(1000);
                    //LiftL.setPower(-.03);
                    //LiftR.setPower(.03);
                    //Thread.sleep(2500);
                    //LiftL.setPower(0);
                    //LiftR.setPower(0);
                    //Thread.sleep(1000);
                    cancelOverride = false;
                    MoveLiftDown = false;
                }
            }
            //RAISE LIFT TO POS 2
            else if(MoveLiftUpPos2){
                if(tensionLift){
                    LiftL.setPower(-.03);
                    LiftR.setPower(.03);
                    Thread.sleep(2500);
                    LiftL.setPower(0);
                    LiftR.setPower(0);
                    Thread.sleep(1500);
                    tensionLift = false;
                }
                if(!tensionLift){
                    if (LiftL.getCurrentPosition() > -1000) {
                        LiftL.setPower(-1);
                    } else {
                        LiftL.setPower(0);
                        MoveLiftUpPos2L = false;
                    }
                    if (LiftR.getCurrentPosition() < 1000) {
                        LiftR.setPower(1);
                    } else {
                        LiftR.setPower(0);
                        MoveLiftUpPos2R = false;
                    }
                    if(!MoveLiftUpPos2L && !MoveLiftUpPos2R){
                        cancelOverride = false;
                        MoveLiftUpPos2 =false;
                    }
                }
            }

            /*

            CODE FOR BALL GRABBER

             */

            //RIGHT BUMPER USED TO TOGGLE BALL GRABBERS IN AND OUT
            //MANUAL OVERRIDE WORKS ON GAMEPAD2'S JOYSTICKS

            boolean isToggled = gamepad2.right_bumper;

            manualOverrideBallG1 = gamepad2.left_stick_y > .25 || gamepad2.left_stick_y < -.25 ? true : false;
            manualOverrideBallG2 = gamepad2.right_stick_y > .25 || gamepad2.right_stick_y < -.25 ? true : false;

            if(manualOverrideBallG1 || manualOverrideBallG2) {
                if(gamepad2.right_stick_y > .25 || gamepad2.left_stick_y > .25) {
                    posBallG1 = BallG1.getPosition() - .001;
                    posBallG2 = BallG2.getPosition() + .001;
                }
                else{
                    posBallG1 = BallG1.getPosition() + .001;
                    posBallG2 = BallG2.getPosition() - .001;
                }
            }
            else{
                posBallG1 = BallG1.getPosition();
                posBallG2 = BallG2.getPosition();
            }

            if(manualOverrideBallG1){
                BallG1.setPosition(posBallG1);
                waitForDetoggle = true;
            }

            if(manualOverrideBallG2){
                BallG2.setPosition(posBallG2);
                waitForDetoggle = true;
            }

            if(isToggled || gamepad2.left_bumper){
                waitForDetoggle = false;
            }

            if(!manualOverrideBallG1 && !manualOverrideBallG2 && !waitForDetoggle) {
                if (isToggled) {
                    rollerState = rollerState != 2 ? rollerState + 1 : 1;
                    Thread.sleep(500);
                }
                if(gamepad2.left_bumper){
                    rollerState = 0;
                }
                //ball grabber 1 reach pos 0 to stow 1 to grab
                //ball grabber 2 reach pos 1 to stow 0 to grab
                if (rollerState == 0) {
                    BallG1.setPosition(.015);
                    BallG2.setPosition(.984);
                } else if (rollerState == 1) {
                    armMoving = true;
                    BallG1.setPosition(0.25);
                    BallG2.setPosition(0.75);
                    if(BallG1.getPosition() == .25 || BallG2.getPosition() == .75){
                        armMoving = false;
                    }
                } else if (rollerState == 2) {
                    armMoving = true;
                    BallG1.setPosition(.35);
                    BallG2.setPosition(.64);
                    if(BallG1.getPosition() == .35 || BallG2.getPosition() == .64){
                        armMoving = false;
                    }
                }
            }

            /*

            BUTTON PUSHER

            */

            if(!buttonInit){
                buttonPusher.setPosition(.5);
                if(gamepad1.y){
                    buttonInit = true;
                }
            }
            else{
                buttonPusher.setPosition(.4);
                Thread.sleep(1500);
                buttonPusher.setPosition(.6);
                Thread.sleep(1500);
                buttonInit = false;
            }

            //MANUAL LAUNCHER OVERRIDE

            if((gamepad1.dpad_up || gamepad1.dpad_down) && !shoot){
                overrideLauncher = true;
            }

            if(overrideLauncher){
                if(gamepad1.dpad_up){
                    LauncherM.setPower(.25);
                }
                else if(gamepad1.dpad_down){
                    LauncherM.setPower(-.25);
                }
                else{
                    LauncherM.setPower(0);
                    EncoderClicks = LauncherM.getCurrentPosition() + 2520;
                    overrideLauncher = false;
                }
            }

            /*
            if(gamepad1.y){
                buttonPusher.setPosition(.4);
                moveButton = true;
                TimeUnit.MILLISECONDS.sleep(5);
                buttonHold = buttonHold + 5;
            }
            else{
                if(moveButton) {
                    buttonPusher.setPosition(.6);
                }
                sleep(buttonHold);
                buttonPusher.setPosition(.5);
                moveButton = false;
                buttonHold = 0;
            }
            */
            /*
            boolean runLoadSequence = gamepad1.left_trigger > .25 ? true : false;

            if(runLoadSequence){
                runReloaderDown = false;
                runReload = true;
            }
            if(runReload){
                Reloader.setPosition(.7);
                if(Reloader.getPosition() == .7){
                    runReloaderDown = true;
                }
                if(runReloaderDown){
                    Reloader.setPosition(.1);
                    if(Reloader.getPosition() == .1) {
                        runReload = false;
                    }
                }
            }
            */
            idle();
        }
    }
}
