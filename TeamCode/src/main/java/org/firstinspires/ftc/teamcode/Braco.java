package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class Braco {

    private DcMotor motorBraco;
    private Servo servoTrava, servoBlock;
    private LinearOpMode opMode;
    private final double[] stages = { Constants.Braco.stage0, Constants.Braco.stage1,
                                      Constants.Braco.stage2, Constants.Braco.stage3 };

    private int stage = 0;
    private double targetPos, adjust = 0, kP, encoder, error, outputPower, reference, setPoint, errorSum;
    public int target, setpoint, pos;

    private ElapsedTime timer;

    private double[] resp = {0, 1, 2, 3, 4, 5, 6, 6, 7, 7};

    public BandejaTeste bandejaTeste;

    public Braco(LinearOpMode opMode){

        this.opMode = opMode;

        motorBraco = opMode.hardwareMap.get(DcMotor.class, "BracoMotor");
        motorBraco.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBraco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoTrava = opMode.hardwareMap.get(Servo.class, "servoTrava");
        servoTrava.setDirection(Servo.Direction.FORWARD);

        servoBlock = opMode.hardwareMap.get(Servo.class, "servoBlock");
        servoBlock.setDirection(Servo.Direction.REVERSE);

        bandejaTeste = new BandejaTeste(this.opMode);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.startTime();

    }

    public void periodic(double adjust) {

        targetPos = stages[stage] + adjust * Constants.Braco.adjust;

        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBraco.setTargetPosition((int)targetPos);
        motorBraco.setPower(0.8);

    }

    /*public void BracoUp(){
            stage = min((stage + 1), 3);
            motorBraco.setPower(.6);
    }
    public void BracoDown(double power){
            stage = max((stage - 1), 0);
            motorBraco.setPower(.6);
    }

    public void PitchBraco(double targetPos, double power) {

        //this.target = target;
        //float lastTarget = target;
        //resetEncoder();

        encoder = motorBraco.getCurrentPosition();
        reference = Constants.Braco.stage0;
        setPoint = Constants.Braco.stage1;

        error = setPoint - encoder;
        outputPower = (kP * error) / 1000;

        // trava para controle e posição do motor
        // Criar variável para armazenar o valor que vai ser passado para a função setTargetPosition
        // Essa variável recebe o valor que vem dos triggers, mas só muda se o valor do trigger for maior que essa variável atual
        // variavel começa com valor 0 e vai aumentando se o valor do trigger é maior que ela própria
        //  Variável reseta (= 0) quando trigger for 0 (piloto soltou botão)


        if (reference <= encoder && encoder <= setPoint) {

            motorBraco.setTargetPosition((int)setPoint);
            motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBraco.setPower(outputPower);

        }
        if (encoder < 0) {
            resetEncoder();
        }


        else if(motorBraco.getCurrentPosition() < Constants.Braco.stage0 || motorBraco.getCurrentPosition() > Constants.Braco.stage1){
            motorBraco.setTargetPosition((int)targetPos);
            motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBraco.setPower(.1);
        }
    }

    public void resetEncoder(){
        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotor setMotorBraco(DcMotor motorBraco){
        this.motorBraco = motorBraco;
        return motorBraco;
    }*/
    public DcMotor getMotorBraco(){
        return this.motorBraco;
    }

    public void travaPos(){
        servoTrava.setPosition(.6);
    }

    public void block(boolean trava){

        pos += (trava ? 1 : 0);
        pos  = Math.max(pos, 0);

        servoBlock.setPosition(trava ? 0 : 1);

    }
    public void pitch(int up, int down){

        pos += up/100 * 10;
        pos -= down/100 * 10;
        pos  = Math.max(pos, 0);

        encoder = motorBraco.getCurrentPosition();

        motorBraco.setTargetPosition(pos);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(encoder >= 300){
            bandejaTeste.destravarBandeja();
        }
        motorBraco.setPower(Math.max(up/100, down/100) * 100);

        //opMode.telemetry.addData("braco", motorBraco.getCurrentPosition());
        //opMode.telemetry.update();
    }

    public void pitchAuto(double power, int target){
        motorBraco.setTargetPosition(target);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(power);
    }

    public void escalar(){

        setpoint = 1350;
        encoder = motorBraco.getCurrentPosition();

        if (encoder < setpoint || encoder > setpoint){

            motorBraco.setTargetPosition(setpoint);
            motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBraco.setPower(.6);

        } else {

            motorBraco.setTargetPosition(0);
            motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBraco.setPower(0);

        }

    }
    public void inverterMotor(){
        motorBraco.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
