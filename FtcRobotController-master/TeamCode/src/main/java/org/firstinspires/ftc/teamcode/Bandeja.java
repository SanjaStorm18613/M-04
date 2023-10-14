package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Bandeja {


    private Servo servoPitch, servoRoll, servoBandeja;
    private boolean buttonBandeja, buttonRollLeft, buttonRollRight;
    private LinearOpMode opMode;
    private float triggerPitch;

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

        if(triggerPitch > 0.1){
            servoPitch.setPosition(triggerPitch);
        }

        if(buttonRollLeft){
            servoRoll.setPosition(-0.5);
        }
        else if(buttonRollRight){
            servoRoll.setPosition(.5);
        }
        else{
            servoRoll.setPosition(0);
        }

        if(buttonBandeja){
            servoBandeja.setPosition(1);
        }
        else{
            servoBandeja.setPosition(0);
        }

    }
}
