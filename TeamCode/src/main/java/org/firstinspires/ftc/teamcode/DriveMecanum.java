package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;

import java.util.Set;

public class DriveMecanum {

    private Servo servoE, servoD;
    private DcMotor FR, FL, BR, BL, Odom_L;
    private DcMotor[] motors;
    private LinearOpMode opMode;
    private ElapsedTime accTime;
    private double acc = 0, x = 0, y = 0, turn = 0, slowFactor = 0;

    public DriveMecanum(LinearOpMode opMode) {

        this.opMode = opMode;



        FR = opMode.hardwareMap.get(DcMotor.class, "FD");
        FL = opMode.hardwareMap.get(DcMotor.class, "FE");
        BR = opMode.hardwareMap.get(DcMotor.class, "BD");
        BL = opMode.hardwareMap.get(DcMotor.class, "BL");

        Odom_L = opMode.hardwareMap.get(DcMotor.class, "LeftOdometry");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        Odom_L.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEnc();

        motors = new DcMotor[]{FL, FR, BR, BL};

        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        accTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        accTime.startTime();

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        setSlowFactor(gamepad1.right_trigger);
    }

    public void periodic() {

        updateAcceleration(Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(turn) < 0.1);

        setDownEncoderServo(true);

        double vel = slowFactor * Constants.DriveMecanum.speed * acc;

        FL.setPower(((y + x) + turn) * vel);
        FR.setPower(((y - x) - turn) * vel);
        BL.setPower(((y - x) + turn) * vel);
        BR.setPower(((y + x) - turn) * vel);
    }

    private void updateAcceleration(boolean release) {

        if (release) {
            acc = 0;
            accTime.reset();
            return;
        }

        acc = Math.min(1, accTime.time() / Constants.DriveMecanum.acceleration);
        acc = Math.round(acc * 1000.0) / 1000.0;
    }

    public void setDownEncoderServo(boolean act) {
        servoE.setPosition(act ? 0 : 1);
        servoD.setPosition(act ? 0 : 1);
    }

    public void resetEnc() {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setSlowFactor(double slowFactor) {
        this.slowFactor = 1 - slowFactor / 1.5;
    }

    public void moveForwardAuto(double power, int target, int odomTarget) {

        Odom_L.setTargetPosition(odomTarget);

        while (Odom_L.getCurrentPosition() < target) {
            for (DcMotor m : motors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setTargetPosition(target);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setPower(power);
            }
        }
    }

    public void turnAuto(double powerFL, double powerBL, int turnTarget, int odomTurnTarget) {

        Odom_L.setTargetPosition(odomTurnTarget);

        while (Odom_L.getCurrentPosition() < turnTarget) {
            for (DcMotor m : motors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setTargetPosition(turnTarget);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            FL.setPower(powerFL);
            FR.setPower(powerBL);
            BL.setPower(powerBL);
            BR.setPower(powerFL);
        }
    }

    public void stopAuto(double power) {
        for (DcMotor m : motors) {
            m.setPower(power);
        }
    }


}
