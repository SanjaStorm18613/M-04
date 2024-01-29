package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class BandejaTeste {


    private Servo servoRoll, servoTravaBandeja, servoPitch;
    private LinearOpMode opMode;

    private DcMotor lampada;

    private double pos;
    private boolean controle = true;

    public BandejaTeste(LinearOpMode opMode){

        this.opMode = opMode;

        servoRoll = opMode.hardwareMap.get(Servo.class, "servoRoll");
        servoRoll.setDirection(Servo.Direction.REVERSE);

        servoTravaBandeja = opMode.hardwareMap.get(Servo.class, "servoTravaBandeja");
        servoTravaBandeja.setDirection(Servo.Direction.FORWARD);

        servoPitch = opMode.hardwareMap.get(Servo.class, "servoPitch");
        servoPitch.setDirection(Servo.Direction.REVERSE);

        lampada = opMode.hardwareMap.get(DcMotor.class, "lampada");
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

    public void pitchBandeja(boolean up){

        /*if (up) {
            servoPitch.setPosition(1);
        }
        if (down) {
            servoPitch.setPosition(0);
        }*/
        pos += (up ? 1 : 0) * 10;
        pos  = Math.max(pos, 0);

        servoPitch.setPosition(up ? 1 : 0);
    }

    public void testePitch(double pos){
        servoPitch.setPosition(pos);
    }

    public Servo getServoTravaBandeja(){
        return servoTravaBandeja;
    }

    public Servo getServoPitch(){
        return servoPitch;
    }

    public DcMotor getLampada(){
        return lampada;
    }

}
