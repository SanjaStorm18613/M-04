package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import android.accounts.AbstractAccountAuthenticator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class LancaDrone {

    private Servo servoDrone, servoPitchDrone;
    private LinearOpMode opMode;
    private boolean buttonLauncher;
    private float pitchTrigger;

    public LancaDrone(LinearOpMode opMode){
        this.opMode = opMode;

        servoDrone = opMode.hardwareMap.get(Servo.class, "Drone");
        servoDrone.setDirection(Servo.Direction.FORWARD);
    }

    public void telemetry(){

    }

    public void lancarDrone(boolean solta){
        servoDrone.setPosition(solta ? 1 : 0);
    }

}