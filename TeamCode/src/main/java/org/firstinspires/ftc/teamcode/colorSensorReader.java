package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by aleja on 3/26/2017.
 */

public class colorSensorReader {

    private final String name;
    private final String name2;
    private I2cDevice ColorR;
    private I2cDevice ColorL;

    public colorSensorReader (String name, String name2, HardwareMap hwmap) {
        this.name = name;
        this.name2 = name2;
        ColorL = hwmap.get(I2cDevice.class, name);
        ColorR = hwmap.get(I2cDevice.class, name2);
        initializeColorSensors();
    }

    I2cDeviceSynch ColorRReader;
    I2cDeviceSynch ColorLReader;

    byte[] ColorRRed;
    byte[] ColorLRed;
    double CRRed;
    double CLRed;

    byte[] ColorRBlue;
    byte[] ColorLBlue;
    double CRBlue;
    double CLBlue;

    public void initializeColorSensors() {
        //Telling the Robot Which I2C Address to Talk To
        ColorRReader = new I2cDeviceSynchImpl(ColorR, I2cAddr.create8bit(0x3a), false);
        ColorLReader = new I2cDeviceSynchImpl(ColorL, I2cAddr.create8bit(0x3c), false);

        //Turning On Color Sensors
        ColorRReader.engage();
        ColorLReader.engage();
    }

    public double[] getColor () {
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

        return new double[]{CLRed, CLBlue, CRRed, CRBlue};
    }
}
