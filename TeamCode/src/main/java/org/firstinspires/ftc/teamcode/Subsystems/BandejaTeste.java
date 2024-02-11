package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class BandejaTeste {

    private Servo servoRoll, servoTravaBandeja, servoPitch, lampada;
    private LinearOpMode opMode;

    int pos = 0, prevpos = 0;
    private DcMotor motorPitch;

    private boolean controle = false;

    public BandejaTeste(LinearOpMode opMode){

        this.opMode = opMode;

        servoRoll = opMode.hardwareMap.get(Servo.class, "servoRoll");
        servoRoll.setDirection(Servo.Direction.REVERSE);

        servoTravaBandeja = opMode.hardwareMap.get(Servo.class, "servoTravaBandeja");
        servoTravaBandeja.setDirection(Servo.Direction.FORWARD);

        servoPitch = opMode.hardwareMap.get(Servo.class, "servoPitch");
        servoPitch.setDirection(Servo.Direction.REVERSE);

        motorPitch = opMode.hardwareMap.get(DcMotor.class, "motorPitch");
        motorPitch.setDirection(DcMotorSimple.Direction.FORWARD);
        motorPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lampada = opMode.hardwareMap.get(Servo.class, "lampada");
        lampada.setDirection(Servo.Direction.FORWARD);

    }

    public void travarBandeja(){
            if (controle) {
                servoTravaBandeja.setPosition(.6);
                lampada.setPosition(1);
            } else if (servoTravaBandeja.getPosition() >= .6 && !controle) {
                servoTravaBandeja.setPosition(.3);
                if(servoTravaBandeja.getPosition() >= .3) lampada.setPosition(0);
            }
            controle = !controle;
    }

    public void travarBandejaTotal() {
        servoTravaBandeja.setPosition(0);
        lampada.setPosition(0);
    }

    public void destravarBandeja() {
        servoTravaBandeja.setPosition(1);
        lampada.setPosition(0);
    }

    public void pitchBandejaMotor(int go, int back){
        //pos += go/100. * 10;
        //pos -= back/100. * 10;
        //pos  = Math.max(pos, 0);

        //calcP = (pos - motorBraco.getCurrentPosition()) * kp;

        if (go == 1){
            pos = 100;
        } else if (back == 1 && go == 0){
            pos = 0;
        }

        motorPitch.setTargetPosition(200);
        motorPitch.setPower(1);

        opMode.telemetry.addData("motorPitch", motorPitch.getCurrentPosition());
        opMode.telemetry.addData("pos", pos);
        opMode.telemetry.addData("x", prevpos);
        opMode.telemetry.addData("a", back);
        opMode.telemetry.addData("trava", servoTravaBandeja.getPosition());
        opMode.telemetry.addData("lampada", lampada.getPosition());
    }

    public void inverterMotorPitchForward(){
        //motorPitch.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void inverterMotorPitchReverse(){
        //motorPitch.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void goBackPitch(){

    }

    public void testePitch(double pos){
        servoPitch.setPosition(pos);
    }

    public Servo getServoTravaBandeja(){
        return servoTravaBandeja;
    }

    public Servo getServoPitch(){
        return servoPitch;
    }



    public DcMotor getMotorPitch(){
        return motorPitch;
    }

    public void lampada(){
        lampada.setPosition(1);
    }

}
