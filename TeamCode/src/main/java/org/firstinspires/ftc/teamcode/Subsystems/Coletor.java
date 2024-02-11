package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Coletor{
    private DcMotor motorCollector;
    private LinearOpMode opMode;
    private int posit, encoder, setPoint = 0, lastError = 0, error;
    private double errorRate = 0, errorSum = 0, lastTime = 0, kP, kD, kI, outputPower, dt;
    private ElapsedTime time, timerToGo;
    private boolean travaPitch = false;

    public Coletor(LinearOpMode opMode){
        this.opMode = opMode;
        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");
        motorCollector.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();

        timerToGo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerToGo.reset();

        timerToGo.startTime();
        time.startTime();

    }

    public void collectorControl(double powerC, double powerR){
        motorCollector.setPower(powerC + powerR);
    }
    public void collectorControlc(boolean a, boolean x) {

        if(a && encoder > -10) {

            setPoint = 140;
            //travaPitch = true;
            encoder = motorCollector.getCurrentPosition();

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

            motorCollector.setPower(outputPower);

            if (encoder >= 139 && encoder <= 141) {
                motorCollector.setPower(0);
            }
        }
        else if(x && encoder != 0){
            setPoint = 0;
            //travaPitch = true;
            encoder = motorCollector.getCurrentPosition();

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

            motorCollector.setPower(outputPower);
            if (encoder >= -4 && encoder <= 2) {
                motorCollector.setPower(0);
            }
        } else {
            motorCollector.setPower(0);
        }

        opMode.telemetry.addData("posit", encoder);
        opMode.telemetry.addData("D", errorRate);
        opMode.telemetry.addData("I", errorSum);

    }

}