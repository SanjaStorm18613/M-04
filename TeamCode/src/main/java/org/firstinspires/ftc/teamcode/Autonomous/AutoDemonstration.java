package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Braco;
import org.firstinspires.ftc.teamcode.Subsystems.Coletor;
import org.firstinspires.ftc.teamcode.Subsystems.DriveMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Lancador;
import org.firstinspires.ftc.teamcode.Subsystems.SistemaLinear;

@Autonomous(name= "AutoDemonstration", group = "Robot")
public class AutoDemonstration extends LinearOpMode {
    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Coletor coletor;
    Braco braco;
    Lancador lancaDrone;
    ElapsedTime timer;
    int step = 0;
    @Override
    public void runOpMode(){
        telemetry.addData("Inicializando auto", " ");

        driveMecanum = new DriveMecanum(this);

        sistemaLinear = new SistemaLinear(this);

        coletor = new Coletor(this);

        braco = new Braco(this);

        lancaDrone = new Lancador(this);

        timer = new ElapsedTime();

        timer.reset();

        while(!isStarted() && !isStopRequested()){
            telemetry.addData("mode", "press play to start");
            telemetry.update();
        }

        while (opModeIsActive()){

            /*if(step == 0) driveMecanum.moveForwardAuto(.6, 1250);
            if(driveMecanum.getBL().getCurrentPosition() >= 1240 && step == 0) resetEnc_step();

            if(step == 1) driveMecanum.turn(.6, 900);
            if(driveMecanum.getBL().getCurrentPosition() >= 890 && step == 1) resetEnc_step();

            if(step == 2) driveMecanum.moveForwardAuto(.4, 70);
            if(driveMecanum.getBL().getCurrentPosition() > 65 && step == 2) resetEnc_step();

            if(step == 3){
                braco.bandeja.setAutoZero();
                driveMecanum.setPowerZero();
                timer.reset();
                timer.startTime();
            }
            if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 3) resetEnc_step();

            if(step == 4) driveMecanum.turn(.7, 1900);
            if(driveMecanum.getBL().getCurrentPosition() > 1790 && step == 4){
                resetEnc_step();
                driveMecanum.setPowerZero();
            }

            if(step == 5) driveMecanum.right(.5, -320);
            if(driveMecanum.getBL().getCurrentPosition() > 310 && step == 5){
                resetEnc_step();
                driveMecanum.setPowerZero();
            }

            if(step == 6) braco.pitchAuto(.6, 2000);
            if(braco.getMotorBraco().getCurrentPosition() > 1990 && step == 6) resetEnc_step();

            if(step == 7) braco.bandeja.pitchAutoCtrl(.4, 100);
            if(braco.bandeja.getMotorPitch().getCurrentPosition() > 90 && step == 7){
                resetEnc_step();
                driveMecanum.setPowerZero();
                timer.reset();
                timer.startTime();
            }

            if(step == 8){
                braco.bandeja.destravarBandejaAuto(.5);
                braco.bandeja.getMotorPitch().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(timer.milliseconds() > 800 && step == 8){
                braco.bandeja.destravarBandejaAuto(.3);
                braco.bandeja.pitchAutoCtrl(.4, 0);
                braco.pitchAuto(.4, 0);
                resetEnc_step();
            }

            if(braco.getMotorBraco().getCurrentPosition() <= 5 && step == 9) driveMecanum.right(.6, 400);
            if(driveMecanum.getBL().getCurrentPosition() < - 390 && step == 9){
                resetEnc_step();
                driveMecanum.setPowerZero();
                lancaDrone.autoLaunch();
            }*/


            /*
            if(step == 0) driveMecanum.moveForwardAuto(.6, 1200);
            if(driveMecanum.getBL().getCurrentPosition() > 1190 && step == 0) resetEnc_step();

            if(step == 1) driveMecanum.moveRight(.6, 1200);
            if(driveMecanum.getBL().getCurrentPosition() < -1190 && step == 1) resetEnc_step();

            if(step == 2) driveMecanum.moveBackwardAuto(.6, 1200);
            if(driveMecanum.getBL().getCurrentPosition() < -1190 && step == 2) resetEnc_step();

            if(step == 3) driveMecanum.moveRight(.6, -1200);
            if(driveMecanum.getBL().getCurrentPosition() > 1190 && step == 3) resetEnc_step();

            if(step == 4){
                driveMecanum.setPowerZero();
                resetEnc_step();
            }*/

            if(step == 0) braco.pitchAuto(.6, 1500);
            if(braco.getMotorBraco().getCurrentPosition() > 1500 && step == 0) {
                resetEnc_step();
            }

            if(step == 1) braco.getMotorBraco().setPower(0);
            if(braco.getMotorBraco().getCurrentPosition() > 1500 && step == 1) {
                braco.bandeja.getMotorPitch().setTargetPosition(65);
                braco.bandeja.getMotorPitch().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                braco.bandeja.getMotorPitch().setPower(.4);
                resetEnc_step();
            }
            if(step == 2 && braco.bandeja.getMotorPitch().getCurrentPosition() > 40) {
                braco.bandeja.getMotorPitch().setPower(0);
                braco.bandeja.destravarBandejaAuto(.5);
            }

            telemetry.addData("braco", braco.getMotorBraco().getCurrentPosition());
            telemetry.addData("step", step);
            telemetry.addData("pitch", braco.bandeja.getMotorPitch().getCurrentPosition());
            telemetry.update();
        }
    }

    public void resetEnc_step(){
        driveMecanum.resetEnc();
        step++;
    }
}
