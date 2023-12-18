package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SistemaLinear {
    private DcMotor armMotor;

    private LinearOpMode opMode;

    private boolean bumperUp, bumperDown, buttonDown;

    private TouchSensor limit;


    private int pos = 0;

    public SistemaLinear(LinearOpMode opMode){

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "MotorBraco");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        limit = opMode.hardwareMap.get(TouchSensor.class, "ArmLimit");

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void periodic(){
        armMotor.setPower(0.8);
        telemetry.addData("pos", pos);
        telemetry.update();
    }

    /**Uso de uma limit para detectar o nível do braço**/
    public void resetEnc(){
        if (!limit.isPressed()){
            armMotor.setPower(0.2);
        } else {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    /**Criando funções para esticar e retrair o sistema, para chama-las no periodic(), sem a necessidade da criação de if's, else's, etc.*/
    public void retrairSistema(boolean Down){
        if (!limit.isPressed()){
            pos -= (Down ? 1 : 0) * 10;
            armMotor.setTargetPosition(pos);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.6);
        }
    }

    public void retrairSistemaTotal(double power, int target){
        if (!limit.isPressed()){
            armMotor.setTargetPosition(target);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(power);
        }
    }

    public void esticarSistema(boolean Up){

        if (armMotor.getCurrentPosition() < 1500){
            pos += (Up ? 1 : 0) * 10;
            armMotor.setTargetPosition(pos);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.6);

        } else {
            armMotor.setPower(0);
        }
    }
}
