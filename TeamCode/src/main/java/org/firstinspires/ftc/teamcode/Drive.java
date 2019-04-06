package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest3_7GearmotorV1;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Drive {

    //Public Members
    DcMotor leftDrive, rightDrive, strafeWheel, wheelMotor;

    VoltageSensor voltageSensor;

    static double ticksPerRot;
    static double maxRPM;

    public static final double wheelCircumfrence = 4 * Math.PI ; //Adjust for Wheel Circum
    public static final int gearRatio = 60; //Adjust for Gear Ratio
    public static final int strafeGearRatio = 40;
    public static final int wheelBase = 14; //Change ASAP

    //Local Members
    HardwareMap map;


    Drive() {

    }

    public void init(HardwareMap hwMap) {
        map = hwMap;

        //Map Motors
        leftDrive = robot7.hardwareMap.get(DcMotor.class, "left");
        rightDrive = robot7.hardwareMap.get(DcMotor.class, "right");

        strafeWheel = robot7.hardwareMap.get(DcMotor.class, "Mid");
        wheelMotor = robot7.hardwareMap.get(DcMotor.class, "MW");

        //Motor Settings

        maxRPM = leftDrive.getMotorType().getMaxRPM();
        ticksPerRot = leftDrive.getMotorType().getTicksPerRev();

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motor Directions

        //Map Servos

        //Servo Settings


    }

    //Implement Methods Here

    boolean driveIsBusy() {
        if (leftDrive.isBusy() == true || rightDrive.isBusy() == true) {
            return true;
        } else {
            return false;
        }

    }


    public void hDrive(String moveType, double inches, double power) {

        moveType = moveType.toLowerCase();

        power = Math.abs(power);

        if (moveType == "forwards" || moveType == "backwards") {
            yMovement(inches, power, moveType);
        }

        if (moveType == "strafe left" || moveType == "strafe right") {
            strafe(inches,power, moveType);
        }

    }



       public void Turn (String moveType, double angle, double power) {

           moveType = moveType.toLowerCase();

           power = Math.abs(power);

           angle = Math.toDegrees(angle);

           double dct = (Math.PI * wheelBase) * (angle/360);

           double wheelRot = (dct/wheelCircumfrence) * ticksPerRot;

           if (moveType == "turn left") {

               leftDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Adjust
               rightDrive.setDirection(DcMotorSimple.Direction.FORWARD); //Adjust

           }

           if (moveType == "turn right") {

               leftDrive.setDirection(DcMotorSimple.Direction.FORWARD); //Adjust
               rightDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Adjust
           }

           leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) wheelRot);
           rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) wheelRot);

           leftDrive.setPower(power);
           rightDrive.setPower(power);

           while (driveIsBusy()) {

           }

           leftDrive.setPower(0);
           rightDrive.setPower(0);


       }

        void strafe (double inches, double power, String moveType){
            moveType.toLowerCase();

            if (moveType == "strafe left") {
                strafeWheel.setDirection(DcMotorSimple.Direction.FORWARD); //Will Probably have to swap
        }
            if (moveType == "strafe right") {
                strafeWheel.setDirection(DcMotorSimple.Direction.REVERSE); //Will Probably have to swap
            }

            strafeWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            strafeWheel.setTargetPosition(strafeWheel.getCurrentPosition() + inchesToTicks(inches, strafeGearRatio));

            strafeWheel.setPower(Math.abs(power));

            while (strafeWheel.isBusy()) {

            }
            strafeWheel.setPower(0);

            strafeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        void yMovement ( double inches, double power, String moveType){

            moveType.toLowerCase();

            if (moveType == "forwards") {
                leftDrive.setDirection(DcMotorSimple.Direction.FORWARD); //Will Probably have to swap
                rightDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Will Probably have to swap
            }
            if (moveType == "backwards") {
                leftDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Will Probably have to swap
                rightDrive.setDirection(DcMotorSimple.Direction.FORWARD); //Will Probably have to swap
            }

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + inchesToTicks(inches, gearRatio));
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + inchesToTicks(inches, gearRatio));

            leftDrive.setPower(Math.abs(power));
            rightDrive.setPower(Math.abs(power));

            while (driveIsBusy()) {

            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        void strafeSetDirection (String setDirection){

            setDirection = setDirection.toLowerCase();

            if (setDirection == "left") {
                strafeWheel.setDirection(DcMotorSimple.Direction.FORWARD); //Will Probably have to swap
                strafeWheel.setDirection(DcMotorSimple.Direction.REVERSE); //Will Probably have to swap
            }

            if (setDirection == "right") {
                strafeWheel.setDirection(DcMotorSimple.Direction.REVERSE); //Will Probably have to swap
                strafeWheel.setDirection(DcMotorSimple.Direction.FORWARD); //Will Probably have to swap
            }
        }

        int inchesToTicks ( double input, int gearR){
            double ticks = ((ticksPerRot * gearR) / wheelCircumfrence) * input;
            return (int) ticks;
        }
    }



