package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Braco {

    private DcMotor motorBraco;
    private Servo servoTrava, servoBlock;
    private LinearOpMode opMode;
    private int stage = 0;
    private double targetPos, adjust = 0, kP, error, outputPower, reference, setPoint, errorSum;
    public int pos, encoder;
    private ElapsedTime timer;
    public BandejaTeste bandejaTeste;

    public Braco(LinearOpMode opMode){

        this.opMode = opMode;

        motorBraco = opMode.hardwareMap.get(DcMotor.class, "BracoMotor");
        motorBraco.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBraco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoTrava = opMode.hardwareMap.get(Servo.class, "servoTrava");
        servoTrava.setDirection(Servo.Direction.REVERSE);

        servoBlock = opMode.hardwareMap.get(Servo.class, "servoBlock");
        servoBlock.setDirection(Servo.Direction.REVERSE);

        bandejaTeste = new BandejaTeste(this.opMode);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.startTime();

        encoder = motorBraco.getCurrentPosition();

    }

    /*public void periodic(double adjust) {

        targetPos = stages[stage] + adjust * Constants.Braco.adjust;

        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBraco.setTargetPosition((int)targetPos);
        motorBraco.setPower(0.8);

    }

    public void PitchBraco(double targetPos, double power) {

        //this.target = target;
        //float lastTarget = target;
        resetEncoder();

        encoder = motorBraco.getCurrentPosition();
        reference = Constants.Braco.stage0;
        setPoint = Constants.Braco.stage1;

        error = setPoint - encoder;
        if(encoder > 1500) outputPower = (kP * error) / 1000;

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
        if (encoder < 0) resetEncoder();


        else if(motorBraco.getCurrentPosition() < Constants.Braco.stage0 || motorBraco.getCurrentPosition() > Constants.Braco.stage1){
            motorBraco.setTargetPosition((int)targetPos);
            motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBraco.setPower(.1);
        }
    }*/

    public void resetEncoder(){
        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public DcMotor getMotorBraco(){
        return this.motorBraco;
    }

    public void travaPos(){
        servoTrava.setPosition(.6);
    }

    /*public void block(boolean trava){

        pos += (trava ? 1 : 0);
        pos  = Math.max(pos, 0);

        servoBlock.setPosition(trava ? 0 : 1);

    }*/
    public void pitch(int up, int down){

        pos += up/100. * 30;
        pos -= down/100. * 10;
        pos  = Math.max(pos, 0);

        //calcP = (pos - motorBraco.getCurrentPosition()) * kp;

        motorBraco.setTargetPosition(pos);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(1);


        opMode.telemetry.addData("encoder", Math.toDegrees(Math.abs(motorBraco.getCurrentPosition()/1000)));
        //opMode.telemetry.update();
    }

    public void pitchAuto(double power, int target){
        motorBraco.setTargetPosition(target);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(power);
    }

    public void inverterMotorForward(){
        motorBraco.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void inverterMotorReverse(){
        motorBraco.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
