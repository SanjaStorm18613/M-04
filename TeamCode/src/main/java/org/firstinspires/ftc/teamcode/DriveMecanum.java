package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;

import java.util.Base64;
import java.util.Set;

public class DriveMecanum {

    private DcMotor FR, FL, BR, BL, odomX, odomY;
    private DcMotor[] motors;
    private LinearOpMode opMode;
    private ElapsedTime accTime;
    //private BNO055IMU imu;
    public double acc = 0, x, y = 0, turn = 0, slowFactor = 0, kP, kI, kD, referenceAngle, integralSum, lastError;
    private int target = 0;

    public DriveMecanum(LinearOpMode opMode) {

        this.opMode = opMode;

        FR = opMode.hardwareMap.get(DcMotor.class, "FR");
        FL = opMode.hardwareMap.get(DcMotor.class, "FL");
        BR = opMode.hardwareMap.get(DcMotor.class, "BR");
        BL = opMode.hardwareMap.get(DcMotor.class, "BL");

        //odomY = opMode.hardwareMap.get(DcMotor.class, "odomY");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

       // odomY.setDirection(DcMotorSimple.Direction.REVERSE);

        /*imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/


        motors = new DcMotor[]{FL, FR, BR, BL};

        resetEnc();

        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        accTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        accTime.startTime();

        setSlowFactor(opMode.gamepad1.right_trigger);
    }

    public void periodic(double x, double y, double turn) {
        this.x = x;

        updateAcceleration(Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(turn) < 0.1);

        double vel = Constants.DriveMecanum.speed * acc;

        FL.setPower(((y - x) - turn) * vel);
        FR.setPower(((y + x) + turn) * vel);
        BL.setPower(((y + x) - turn) * vel);
        BR.setPower(((y - x) + turn) * vel);

        //odomY.setTargetPosition((int)y);
    }

    public void updateAcceleration(boolean release) {

        if (release) {
            acc = 0;
            accTime.reset();
            return;
        }

        acc = Math.min(1, accTime.time() / Constants.DriveMecanum.acceleration);
        acc = Math.round(acc * 1000.) / 1000.;
    }

    public void resetEnc() {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //odomY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //odomY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setSlowFactor(double slowFactor) {
        this.slowFactor = 1 - slowFactor / 1.5;
    }

    public void moveForwardAuto(double power, int target) {
        //power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
        for (DcMotor m : motors) {
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);

        }
    }
    public void moveBackwardAuto(double power, int target) {
        //power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
        for (DcMotor m : motors) {
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(-power);
        }
    }

    public void runToPosition() {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void turn(double power, int target) {

        //resetEnc();

        FR.setTargetPosition(-target);
        BR.setTargetPosition(-target);
        FL.setTargetPosition(target);
        BL.setTargetPosition(target);

        runToPosition();

        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(power);
        BR.setPower(-power);

    }
    public void setPowerZero() {

        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        BR.setPower(0);

    }

    public void right(double power, int target) {

        BL.setTargetPosition(-target);
        BR.setTargetPosition(target);
        FL.setTargetPosition(target);
        FR.setTargetPosition(-target);

        runToPosition();

        BL.setPower(-power);
        BR.setPower(power);
        FL.setPower(power);
        FR.setPower(-power);

    }

   /* public double PIDControl(double reference, double state) {

        double error = angleWrap(reference - state);
        integralSum += error * accTime.milliseconds();
        double derivative = (error - lastError) / accTime.milliseconds();
        error = lastError;
        accTime.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;

    }

    public double angleWrap(double radians){
        while(radians > Math.PI){ radians -= 2 * Math.PI; }

        while(radians < -Math.PI){ radians += 2 * Math.PI; }

        return radians;
    }*/

    public DcMotor getBL(){
        return this.BL;
    }
    public DcMotor setBL(DcMotor BL){
        this.BL = BL;
        return BL;
    }

    public DcMotor getOdomX(){
        return odomX;
    }
    public DcMotor getOdomY(){
        return this.odomY;
    }
    public double setX(double x){
        this.x = x;
        return x;
    }

    public DcMotor setOdomY(DcMotor odomY){
        this.odomY = odomY;
        return  odomY;
    }
}