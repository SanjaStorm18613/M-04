package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bandeja {

    private Servo servoTravaBandeja, travaAutonomo;
    private LinearOpMode opMode;
    private int pos = 0;
    private DcMotor motorPitch;
    private int encoder, setPoint = 0, lastError = 0, error;
    private double errorRate = 0, errorSum = 0, lastTime = 0, kP, kD, kI, outputPower, dt;
    private ElapsedTime time, timerToGo;
    public Coletor coletor;
    private boolean isLocked = false, isUnlocked = false;

    public Bandeja(LinearOpMode opMode){

        this.opMode = opMode;

        coletor = new Coletor(this.opMode);

        servoTravaBandeja = opMode.hardwareMap.get(Servo.class, "servoTravaBandeja");
        servoTravaBandeja.setDirection(Servo.Direction.FORWARD);

        travaAutonomo = opMode.hardwareMap.get(Servo.class, "travaAutonomo");
        travaAutonomo.setDirection(Servo.Direction.FORWARD);

        motorPitch = opMode.hardwareMap.get(DcMotor.class, "motorPitch");
        motorPitch.setDirection(DcMotorSimple.Direction.FORWARD);
        motorPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();

        timerToGo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerToGo.reset();

        timerToGo.startTime();
        time.startTime();

    }

    public void bandejaControl(){
        if (isLocked) {
            servoTravaBandeja.setPosition(.5);
        } else {
            servoTravaBandeja.setPosition(.3);
        }

        isLocked = !isLocked;
    }

    public void bandejaColetorControl(){

        if (coletor.getMotorCollector().getPower() > .3 || coletor.getMotorCollector().getPower() < - 0.8) {
            servoTravaBandeja.setPosition(.5);
            isUnlocked = true;
        } else if (isUnlocked) {
            servoTravaBandeja.setPosition(.3);
            isUnlocked = false;
        }
        
    }
    public Servo getServoTravaBandeja(){
        return servoTravaBandeja;
    }
    public DcMotor getMotorPitch(){
        return motorPitch;
    }
    public void pitchControl(boolean a, boolean x) {

        if(a && encoder > -10) {

            setPoint = 180;
            encoder = motorPitch.getCurrentPosition();

            error = setPoint - encoder;

            kP = .08;
            kD = .0005;
            kI = .0000000006;

            errorRate = ((error - lastError) / dt);
            lastError = error;
            lastTime = time.time();

            dt = time.time() - lastTime;
            errorSum += error * dt;

            outputPower = error * kP + kI * errorSum + kD * errorRate;

            motorPitch.setPower(outputPower);

            if (encoder >= 179 && encoder <= 181) {
                motorPitch.setPower(0);
            }
        }

        else if(x && encoder != 0){

            setPoint = -5;
            encoder = motorPitch.getCurrentPosition();

            error = setPoint - encoder;

            kP = .08;
            kD = .0005;
            kI = .0000000006;

            errorRate = ((error - lastError) / dt);
            lastError = error;
            lastTime = time.time();

            dt = time.time() - lastTime;
            errorSum += error * dt;

            outputPower = error * kP + kI * errorSum + kD * errorRate;

            motorPitch.setPower(outputPower * .5);
            if (encoder >= -4 && encoder <= 2) {
                motorPitch.setPower(0);
            }

        } else {
            motorPitch.setPower(0);
        }

        opMode.telemetry.addData("posit", encoder);
        opMode.telemetry.addData("D", errorRate);
        opMode.telemetry.addData("I", errorSum);
        opMode.telemetry.addData("trava", servoTravaBandeja.getPosition());
        opMode.telemetry.addData("motorPitch", motorPitch.getCurrentPosition());
        opMode.telemetry.addData("pos", pos);
        //opMode.telemetry.addData("lampada", lampada.getPosition());
    }

    public void travaAutonomo(){
        travaAutonomo.setPosition(.4);
    }

    public Servo getTravaAuto(){
        return travaAutonomo;
    }

}
