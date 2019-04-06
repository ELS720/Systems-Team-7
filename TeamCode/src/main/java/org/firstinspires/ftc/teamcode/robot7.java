package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class robot7 {

    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;

    Drive drive;
    IMU imu;

    public robot7(HardwareMap hardwareMap, Telemetry telemetry) {
        robot7.hardwareMap = hardwareMap;
        robot7.telemetry = telemetry;

        drive = new Drive();
        imu = new IMU();

    }
}
