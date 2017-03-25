package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by aleja on 3/24/2017.
 */

public class sonarReader {

    private final String name;
    private final String name2;
    private I2cDevice SonarR;
    private I2cDevice SonarL;

    public sonarReader(String name, String name2, HardwareMap hwmap){
        this.name = name;
        this.name2 = name2;
        SonarL = hwmap.get(I2cDevice.class, name);
        SonarR = hwmap.get(I2cDevice.class, name2);
        initializeSonar();
    }

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

    //Switches for Taking Distance Snapshots
    boolean CollectDistanceTime = false;

    //Defining Sleep Method Variables
    double Sleep = 0;
    double WakeUpTime;
    double SleepEnable = 0;

    I2cDeviceSynch SonarRReader;
    I2cDeviceSynch SonarLReader;

    public void initializeSonar () {

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

        if (SleepEnable == 0) {

            WakeUpTime = System.currentTimeMillis() + Sleep;

            while (System.currentTimeMillis() < WakeUpTime) {

            }

            Sleep = 0;

            SleepEnable = 1;

        }
    }

    public double[] getDistances () {

        boolean StartRead = false;
        boolean LookForValue = false;
        boolean RecordValue = false;

        //Save the High and Low Bytes for the Right Sensor from the Last Snapshot
        RightDistanceTimeH = SonarRReader.read(0x02, 1);
        RightDistanceTimeL = SonarRReader.read(0x03, 1);

        //Save the High and Low Bytes for the Left Sensor from the Last Snapshot
        LeftDistanceTimeH = SonarLReader.read(0x02, 1);
        LeftDistanceTimeL = SonarLReader.read(0x03, 1);

        CollectDistanceTime = true;

        //Reading the Sonar Sensors

        //Reading Distance in MicroSeconds
        if (CollectDistanceTime == true)//Once the CollectDistanceTime boolean is switched on, the robot takes a snapshot of the distance in MicroSeconds
        {

            StartRead = true;


            CollectDistanceTime = false;
        }

        if(StartRead == true) {
            //Command the Sonars to Take a Snapshot
            SonarRReader.write8(0, 82);
            SonarLReader.write8(0, 82);

            LookForValue = true;
            StartRead = false;

        }

        if(LookForValue == true)
        {

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

        if(((RightDistanceTimeHDouble != 255) && (RightDistanceTimeLDouble != 255)) || ((LeftDistanceTimeHDouble != 255) && (LeftDistanceTimeLDouble != 255)))
        {

            LookForValue = false;
            RecordValue = true;

        }

        if(RecordValue == true) {

            //Save Full Distance Values from Last Snapshot
            RightDistanceTime = (RightDistanceTimeHDouble * 256) + RightDistanceTimeLDouble;
            LeftDistanceTime = (LeftDistanceTimeHDouble * 256) + LeftDistanceTimeLDouble;
        }

        return new double[]{LeftDistanceTime, RightDistanceTime};
    }
}
