package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Braco.stage1;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bandeja {

    private Servo servoPitch, servoRoll, servoBandeja;
    private LinearOpMode opMode;
    private double triggerPitch, stage = 0;

    private final double[] pitchStages = {Constants.PitchBandeja.pitchStage1, Constants.PitchBandeja.pitchStage2},
                                  drop = {.1, 0, .5};
    private int adjust = 0, count_drop = -1, cont = 1;

    public Bandeja(LinearOpMode opMode){

        this.opMode = opMode;

        servoPitch = opMode.hardwareMap.get(Servo.class, "PitchServo");
        servoPitch.setDirection(Servo.Direction.REVERSE);

        servoRoll = opMode.hardwareMap.get(Servo.class, "RollServo");
        servoRoll.setDirection(Servo.Direction.FORWARD);

        servoBandeja = opMode.hardwareMap.get(Servo.class, "ServoBandeja");
        servoBandeja.setDirection(Servo.Direction.FORWARD);

        /*bandejaTelemetry = opMode.telemetry;
        bandejaTelemetry.addData("Bandeja", servoBandeja.getPosition());
        bandejaTelemetry.update();*/
    }

    public void periodic(){ }

    public void rollBandeja(int signum){
        cont = (cont + 2) % 4;
        servoRoll.setPosition(signum*(cont-1));

        //servoRoll.setPosition(++cont % 2);
    }

    public void destravarBandeja(){
        count_drop += Math.min(cont,2);
        servoBandeja.setPosition(drop[count_drop % 3]);
        opMode.telemetry.addData("X", count_drop % 3);
        opMode.telemetry.update();
    }

    public void travarBandeja(){
        servoBandeja.setPosition(0);
    }
    public void destravarBandejaTotal(){
        servoBandeja.setPosition(.8);
    }

    //public void pitchBandeja(double alpha, double adjust){
      //  servoPitch.setPosition(.33 - alpha/180 + adjust/180);
    //}*/

    public Servo getServoBandeja(){
        return this.servoBandeja;
    }
    public Servo setServoBandeja(Servo servoBandeja){
        this.servoBandeja = servoBandeja;
        return servoBandeja;
    }

    public void pitchBandeja(){
        servoPitch.setPosition(1);
    }
    public void bandejaV(){
        servoPitch.setPosition(0);
    }


}
