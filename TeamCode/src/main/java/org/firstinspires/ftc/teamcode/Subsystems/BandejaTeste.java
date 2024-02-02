package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class BandejaTeste {

    private Servo servoRoll, servoTravaBandeja, servoPitch;
    private LinearOpMode opMode;

    private DcMotor motorPitch;

    private double pos;
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
    }

    public void rollBandeja(boolean left, boolean right){

        if(left && servoRoll.getPosition() == 0){
            servoRoll.setPosition(1);
            controle = true;
        }

        if(right && servoRoll.getPosition() == 1){
            servoRoll.setPosition(0);
            controle = true;
        }
    }

    public void travarBandeja(){
            if (controle) {
                servoTravaBandeja.setPosition(.6);
            } else {
                servoTravaBandeja.setPosition(.3);
            }
            controle = !controle;
    }

    public void travarBandejaTotal() {
        servoTravaBandeja.setPosition(0);
    }

    public void destravarBandeja() {
        servoTravaBandeja.setPosition(1);
    }

    public void pitchBandeja(boolean up){
        pos += (up ? 1 : 0);
        pos  = Math.max(pos, 0);

        servoPitch.setPosition(up ? .6 : 0);
    }
    public void pitchBandejaMotor(boolean go, boolean back){
        if(go){
            //motorPitch.setDirection(DcMotorSimple.Direction.REVERSE);
            motorPitch.setPower(.4);
        } else if (back){
            //motorPitch.setDirection(DcMotorSimple.Direction.FORWARD);
            motorPitch.setPower(-.4);
        } else {
            motorPitch.setPower(0);
        }
    }

    public void inverterMotorPitchForward(){
        //motorPitch.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void inverterMotorPitchReverse(){
        //motorPitch.setDirection(DcMotorSimple.Direction.REVERSE);
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

}
