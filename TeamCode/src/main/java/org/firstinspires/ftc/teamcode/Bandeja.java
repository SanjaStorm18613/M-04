package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Braco.stage1;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Bandeja {


    private Servo servoPitch, servoRoll, servoBandeja;
    private boolean buttonBandeja, buttonBandejaLeft, buttonBandejaRight, buttonRollLeft, buttonRollRight, buttonPitch, auto_pitch;
    private LinearOpMode opMode;
    private double triggerPitch;
    private final double[] pitchStages = {Constants.PitchBandeja.pitchStage1, Constants.PitchBandeja.pitchStage2},
                            drop = {0,1,2};
    private double stage = 0;
    private int adjust = 0, count_drop = 0;

    public Bandeja(LinearOpMode opMode){

        this.opMode = opMode;

        servoPitch = opMode.hardwareMap.get(Servo.class, "PitchServo");
        servoPitch.setDirection(Servo.Direction.FORWARD);

        servoRoll = opMode.hardwareMap.get(Servo.class, "RollServo");
        servoRoll.setDirection(Servo.Direction.FORWARD);

        servoBandeja = opMode.hardwareMap.get(Servo.class, "ServoBandeja");
        servoBandeja.setDirection(Servo.Direction.FORWARD);

        triggerPitch = opMode.gamepad2.right_trigger;
        buttonRollLeft = opMode.gamepad2.dpad_left;
        buttonRollRight = opMode.gamepad2.dpad_right;
        buttonBandeja = opMode.gamepad1.b;

    }

    public void periodic(){

        if(buttonPitch){
        }

        if(triggerPitch > .1){
            servoPitch.setPosition(triggerPitch);
        }
        else{
            servoPitch.setPosition(0);
        }

        if(buttonRollLeft){
            servoRoll.setPosition(-.5*(++cont % 2));
        }
        else if(buttonRollRight){
            servoRoll.setPosition(.5);
        }

        if(buttonBandeja){
            servoBandeja.setPosition(Math.signum(servoPitch.getPosition())*drop[++count_drop % 2]);
        }


        // linha para garantir que o piloto não solte os pixels por engano;
        // supondo que o valor -1 destrave o pixel da esquerda
        // e que o valor 1 destrave o pixel da direita
        // e que 0 mantenha os dois pixels travados
        if(buttonBandejaLeft){
            if(servoRoll.getPosition() >= -0.55 && servoRoll.getPosition() <= -0.45) {
                servoBandeja.setPosition(-1);
            }
        }
    }
}
