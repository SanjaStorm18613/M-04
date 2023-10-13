package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import android.accounts.AbstractAccountAuthenticator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class LancaDrone {

    private Servo servoDrone;
    private LinearOpMode opMode;
    private boolean button;

    public LancaDrone(LinearOpMode opMode){
        this.opMode = opMode;
        servoDrone = opMode.hardwareMap.get(Servo.class, "Drone");
        setDispararDrone(gamepad2.a);
        servoDrone.setDirection(Servo.Direction.FORWARD);
    }

    public void periodic(){
        if(button){
            servoDrone.setPosition(.6);
        }
    }
    public void setDispararDrone(boolean gButton){
        button = gButton;
    }
}
