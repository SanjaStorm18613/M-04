package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class BandejaTeste {


    private Servo servoRoll, servoTravaBandeja, servoPitch;
    private LinearOpMode opMode;

    Braco braco;

    private double pos;
    private boolean controle = true;

    public BandejaTeste(LinearOpMode opMode){

        this.opMode = opMode;

        braco = new Braco(this.opMode);

        servoRoll = opMode.hardwareMap.get(Servo.class, "servoRoll");
        servoRoll.setDirection(Servo.Direction.REVERSE);

        servoTravaBandeja = opMode.hardwareMap.get(Servo.class, "servoTravaBandeja");
        servoTravaBandeja.setDirection(Servo.Direction.FORWARD);

        servoPitch = opMode.hardwareMap.get(Servo.class, "servoPitch");
        servoPitch.setDirection(Servo.Direction.REVERSE);
    }

    public void rollBandeja(boolean left, boolean right){

        if(left && servoRoll.getPosition() == 0){
            servoRoll.setPosition(1);
            controle = true;
        }

        if(right && servoRoll.getPosition() == 1){
            servoRoll.setPosition(0);
            controle = true;
        }

        opMode.telemetry.addData("servo", servoTravaBandeja.getPosition());
        opMode.telemetry.update();
    }

    public void destravarBandeja(){

        if (servoRoll.getPosition() == 1) {
            if (controle) {
                servoTravaBandeja.setPosition(.6);
            } else  {
                servoTravaBandeja.setPosition(.3);
            }
            controle = !controle;
        } else {
            if (controle) {
                servoTravaBandeja.setPosition(.3);
            } else {
                servoTravaBandeja.setPosition(.6);
            }
            controle = !controle;
        }
    }

    public void destravarBandejaTotal() {
        servoTravaBandeja.setPosition(0);
    }

    public void travarBandeja() {
        servoTravaBandeja.setPosition(1);
    }

    public void pitchBandeja(boolean up, boolean down){

        if (up) {
            servoPitch.setPosition(1);
        }
        if (down) {
            servoPitch.setPosition(.1);
        }
    }

    public void pitch(boolean up, boolean down){

        if(braco.getMotorBraco().getCurrentPosition() > 0 && braco.getMotorBraco().getCurrentPosition() < 200){
            servoPitch.setPosition(0);
        } else if (up && braco.getMotorBraco().getCurrentPosition() > 201) {
            servoPitch.setPosition(1);
        } else if (down && braco.getMotorBraco().getCurrentPosition() > 201) {
            servoPitch.setPosition(.1);
        }

    }

    public void testePitch(){
        servoPitch.setPosition(0);
    }

    public Servo getServoTravaBandeja(){
        return servoTravaBandeja;
    }



}
