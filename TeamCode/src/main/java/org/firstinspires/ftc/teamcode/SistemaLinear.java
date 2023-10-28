package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.Constants.Pitch.PITCH_LOW;
import static org.firstinspires.ftc.teamcode.Constants.Pitch.PITCH_UP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Base64;

public class SistemaLinear {
    private DcMotor armMotor;
    private LinearOpMode opMode;
    private Servo servoGancho, servoPitchBraco;
    private boolean bumperUp, bumperDown, buttonDown, buttonGanchoUp, bumperPitchBracoL;

    private int pos = 0;

    public SistemaLinear(LinearOpMode opMode){

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "MotorBraco");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoGancho = opMode.hardwareMap.get(Servo.class, "ServoGANCHO");
        servoGancho.setDirection(Servo.Direction.FORWARD);

        servoPitchBraco = opMode.hardwareMap.get(Servo.class, "ServoPitchBraco");
        servoPitchBraco.setDirection(Servo.Direction.FORWARD);

        bumperUp = opMode.gamepad2.left_bumper;
        bumperDown = opMode.gamepad2.right_bumper;
        buttonDown = opMode.gamepad2.x;
        buttonGanchoUp = opMode.gamepad1.y;
        bumperPitchBracoL = opMode.gamepad1.right_bumper;

    }

    public void periodic(){

        //Retrair o sistema
        if(bumperDown && armMotor.getCurrentPosition() > 30) {
            armMotor.setTargetPosition(30);
        }
        //Esticar o sistema
        else if(bumperUp && armMotor.getCurrentPosition() < 150) {
            armMotor.setTargetPosition(pos++);
        } else if(buttonDown && armMotor.getCurrentPosition() > 30){
            armMotor.setTargetPosition(pos--);
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
    }

    /*public void setArmEncoder(double power){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }*/

}
