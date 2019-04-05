package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest3_7GearmotorV1;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class HardwareRobot7 {

//Public Members
    DcMotor leftDrive, rightDrive, strafeWheel, wheelMotor;

    VoltageSensor voltageSensor;

    static int ticksPerRot;
    static int maxRPM;
    //double outputvoltage;
    public static final double wheelCircumfrence = 4.0; //Change value

 //Local Members
    HardwareMap map;


    HardwareRobot7() {

    }

    public void init (HardwareMap hwMap) {
        map = hwMap;

        //Map Motors
        leftDrive = map.get(DcMotor.class, "left");
        rightDrive = map.get(DcMotor.class, "right");

        strafeWheel = map.get(DcMotor.class, "Mid");
        wheelMotor = map.get(DcMotor.class, "FW");

        //Motor Settings
        ticksPerRot = (int) leftDrive.getMotorType().getTicksPerRev();
        maxRPM = (int) leftDrive.getMotorType().getMaxRPM();
        //Set Motor Directions

        //Map Servos

        //Servo Settings


    }

    //Implement Methods Here

   }

