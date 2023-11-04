package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Braco.stage1;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Bandeja {


    private Servo servoPitch, servoRoll, servoBandeja;
    private boolean buttonBandeja, buttonBandeja2, buttonBandejaLeft, buttonBandejaRight, buttonRollLeft, buttonRollRight, buttonPitch, auto_pitch;
    private LinearOpMode opMode;
    private double triggerPitch, stage = 0;
    private final double[] pitchStages = {Constants.PitchBandeja.pitchStage1, Constants.PitchBandeja.pitchStage2},
                            drop = {0,1,2};
    private int adjust = 0, count_drop = 0, cont = 0;

    public Bandeja(LinearOpMode opMode){

        this.opMode = opMode;
        this.servoPitch = servoPitch;

        servoPitch = opMode.hardwareMap.get(Servo.class, "PitchServo");
        servoPitch.setDirection(Servo.Direction.FORWARD);

        servoRoll = opMode.hardwareMap.get(Servo.class, "RollServo");
        servoRoll.setDirection(Servo.Direction.FORWARD);

        servoBandeja = opMode.hardwareMap.get(Servo.class, "ServoBandeja");
        servoBandeja.setDirection(Servo.Direction.FORWARD);

    }

    public void periodic(){

    }

    public void rollBandeja(int signum){
        servoRoll.setPosition(signum*.5*(++cont % 2));

        //servoRoll.setPosition(++cont % 2);
    }

    public void destravarBandeja(){
        servoBandeja.setPosition(Math.signum(servoPitch.getPosition())*drop[++count_drop % 2]);
    }

    public void destravarBandejaTotal(){
            servoBandeja.setPosition(.5);
    }

    public void pitchBandeja(double alpha, double adjust){
        servoPitch.setPosition(.67 - alpha/180 + adjust/180);
    }
}
