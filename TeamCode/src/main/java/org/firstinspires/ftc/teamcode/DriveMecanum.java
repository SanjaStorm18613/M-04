package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    private Servo[] servos;
    private DcMotor FR, FL, BR, BL;
    private DcMotor[] motors;
    private LinearOpMode opMode;
    private ElapsedTime accTime;
    private double acc = 0, x = 0, y = 0, turn = 0, slowFactor = 0;
    private int target = 0;

    public DriveMecanum(LinearOpMode opMode) {

        this.opMode = opMode;


        FR = opMode.hardwareMap.get(DcMotor.class, "FR");
        FL = opMode.hardwareMap.get(DcMotor.class, "FL");
        BR = opMode.hardwareMap.get(DcMotor.class, "BR");
        BL = opMode.hardwareMap.get(DcMotor.class, "BL");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        servoD = opMode.hardwareMap.get(Servo.class, "RightOdometry");
        servoE = opMode.hardwareMap.get(Servo.class, "LeftOdometry");


        motors = new DcMotor[]{FL, FR, BR, BL};
        servos = new Servo[]{servoD, servoE};

        resetEnc();

        for(Servo s : servos){
            s.setDirection(Servo.Direction.FORWARD);
        }

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

    public void periodic(){

        updateAcceleration(Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(turn) < 0.1);

        setDownEncoderServo(true);

        double vel = slowFactor * Constants.DriveMecanum.speed * acc;

        FL.setPower(((y + x) + turn) * vel);
        FR.setPower(((y - x) - turn) * vel);
        BL.setPower(((y - x) + turn) * vel);
        BR.setPower(((y + x) - turn) * vel);

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", gamepad1.left_stick_y);
        telemetry.addData("Velocidade", vel);
        telemetry.addData("Turn", turn);
        telemetry.update();
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

    public void moveForwardAuto(double power, int target){

        for (DcMotor m : motors) {

            m.setTargetPosition(target);
            runToPosition();
            m.setPower(power);

        }
    }

    public void runToPosition(){

        for(DcMotor m : motors){

            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

    }


    public void turn(double powerF, int target){

        resetEnc();

        FR.setTargetPosition(-target);
        BL.setTargetPosition(-target);
        BR.setTargetPosition(target);
        FL.setTargetPosition(target);

        runToPosition();

        FR.setPower(-powerF);
        BL.setPower(-powerF);
        FL.setPower(powerF);
        BR.setPower(powerF);

    }

    public void right(double power, int target){

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

    public DcMotor getBL(){
        return this.BL;
    }
    public DcMotor setBL(DcMotor BL){
        this.BL = BL;
        return BL;
    }

}



