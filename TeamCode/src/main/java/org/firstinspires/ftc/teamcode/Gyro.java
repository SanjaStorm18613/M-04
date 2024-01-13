package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {

    private BNO055IMU imu;
    private double lastAngle;
    private int revolutions;


    public Gyro(LinearOpMode opMode){

        BNO055IMU.Parameters GYRO_imu_parameters;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu2");

        GYRO_imu_parameters = new BNO055IMU.Parameters();
        GYRO_imu_parameters.mode = BNO055IMU.SensorMode.IMU;
        GYRO_imu_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        GYRO_imu_parameters.loggingEnabled = false;

        lastAngle = 0;

        imu.initialize(GYRO_imu_parameters);

        revolutions = 0;
    }

    public double getContinuousAngle() {

        double currentAngle = getPeriodicAngle();

        if ((Math.abs(lastAngle - currentAngle)) > 180) {

            if (lastAngle > currentAngle) {
                revolutions++;
            } else {
                revolutions--;
            }

        }
        lastAngle = currentAngle;

        return revolutions * 360 + currentAngle;

    }

    public double getPeriodicAngle(){

        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }
}
