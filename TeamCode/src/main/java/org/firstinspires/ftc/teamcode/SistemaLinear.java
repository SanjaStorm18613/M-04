package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SistemaLinear {
    private DcMotor armMotor;

    private LinearOpMode opMode;

    private Servo servoPitchBraco;

    private boolean bumperUp, bumperDown, buttonDown;

    private TouchSensor limit;

    Telemetry systemTelemetry;

    private int pos = 0;

    public SistemaLinear(LinearOpMode opMode){

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "MotorBraco");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoPitchBraco = opMode.hardwareMap.get(Servo.class, "ServoPitchBraco");
        servoPitchBraco.setDirection(Servo.Direction.FORWARD);

        limit = opMode.hardwareMap.get(TouchSensor.class, "ArmLimit");

        systemTelemetry = opMode.telemetry;
        systemTelemetry.addData("pos", pos);
        systemTelemetry.update();

    }

    public void periodic(){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
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
        }
    }

    public void retrairSistemaTotal(){
        if (!limit.isPressed()){
            armMotor.setTargetPosition(30);
        }
    }

    public void esticarSistema(boolean Up){
        if (armMotor.getCurrentPosition() < 150){
            pos += (Up ? 1 : 0) * 10;
            armMotor.setTargetPosition(pos);
        } else {
            armMotor.setPower(0);
        }
    }
}
