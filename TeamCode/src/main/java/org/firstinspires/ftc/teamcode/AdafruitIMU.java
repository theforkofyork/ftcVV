package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.LBHW;


/**
 * Created by Archish on 10/28/16.
 */

public class AdafruitIMU {
    private final BNO055IMU imu;
    private final String name;

    public AdafruitIMU(String name) {
        this.name = name;
        imu = FtcOpModeRegister.opModeManager.getHardwareMap().get(BNO055IMU.class, name);
        setParameters();
    }

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }


    public double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        double roll = Math.atan2( 2*(w*x + y*z) , 1 - 2*(x*x + y*y) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }
    public double adjustAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    } //puts angle from 0-360 to -180 to 180
    public double getHeading() {
        return getAngles()[0];
    }
    public double getPitch() {
        return getAngles()[1];
    }
    public double getRoll() {
        return getAngles()[2];
    }

    public void telemetryRun() {
    }

}