package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class BandejaTeste {


    private Servo servoRoll, servoTravaBandeja, servoPitch;
    private LinearOpMode opMode;

    private double pos;
    private boolean trava = false, travaTotal = false, travaBandeja = false;

    public BandejaTeste(LinearOpMode opMode){

        this.opMode = opMode;

        servoRoll = opMode.hardwareMap.get(Servo.class, "servoRoll");
        servoRoll.setDirection(Servo.Direction.FORWARD);

        servoTravaBandeja = opMode.hardwareMap.get(Servo.class, "servoTrava");
        servoTravaBandeja.setDirection(Servo.Direction.FORWARD);

        servoPitch = opMode.hardwareMap.get(Servo.class, "servoPitch");
        servoPitch.setDirection(Servo.Direction.REVERSE);
    }

    public void rollBandeja(boolean left, boolean right){

        if(left && servoRoll.getPosition() == 0) {
            servoRoll.setPosition(1);
        }

        if(right && servoRoll.getPosition() == 1){
            servoRoll.setPosition(0);
        }
    }

    public void destravarBandeja(boolean trava){

        this.trava = trava;

        if(servoRoll.getPosition() == 1 && trava) {
            servoTravaBandeja.setDirection(Servo.Direction.FORWARD);
            servoTravaBandeja.setPosition(trava ? .3 : 0);
        }
        if(servoRoll.getPosition() == 0 && trava) {
            servoTravaBandeja.setDirection(Servo.Direction.REVERSE);
            servoTravaBandeja.setPosition(trava ? .5 : 0);
        }
    }

    public void destravarBandejaTotal(boolean travaTotal) {

        this.travaTotal = travaTotal;

        if(travaTotal && (servoRoll.getPosition() == 0 || servoRoll.getPosition() == 1)){

            servoTravaBandeja.setPosition(1);

        }
    }

    public void travarBandeja(boolean travaBandeja) {

        this.travaBandeja = travaBandeja;

        if(travaBandeja){
            servoTravaBandeja.setPosition(0);
        }
    }
}
