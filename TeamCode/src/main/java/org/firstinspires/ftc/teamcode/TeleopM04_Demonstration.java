package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Braco;
import org.firstinspires.ftc.teamcode.Subsystems.Coletor;
import org.firstinspires.ftc.teamcode.Subsystems.DriveMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Lancador;
import org.firstinspires.ftc.teamcode.Subsystems.SistemaLinear;

@TeleOp(name = "TeleopM04_Demo", group = "Linear OpMode")

public class TeleopM04_Demonstration extends LinearOpMode {
    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Lancador lancador;
    Braco braco;
    private boolean bandejaBlock = false, inverterMotorElevador = false;
    private ElapsedTime riggingTime;

    @Override
    public void runOpMode() {

        driveMecanum  = new DriveMecanum (this);
        sistemaLinear = new SistemaLinear(this);
        lancador      = new Lancador     (this);
        braco         = new Braco        (this);

        riggingTime   = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        riggingTime.reset();

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {

                //LANCADOR
                lancador.droneSetZero(gamepad2.dpad_up);
                lancador.launchDrone(gamepad2.y);

                //COLETOR
                braco.bandeja.coletor.collectorControl(gamepad1.left_trigger, -gamepad1.right_trigger);

                //BRACO
                braco.armControl((int) gamepad2.left_trigger * 100, (int) gamepad2.right_trigger * 100);

                //SISTEMA LINEAR
                sistemaLinear.elevatorControl(gamepad1.left_bumper, gamepad1.right_bumper);
                sistemaLinear.setMode(gamepad1.back);

                inverterMotorElevador = gamepad1.back;

                //BANDEJA
                if (gamepad2.a && !bandejaBlock) braco.bandeja.bandejaControl();

                braco.bandeja.bandejaColetorControl();

                bandejaBlock = gamepad2.a;
                braco.bandeja.pitchControl(gamepad1.a, gamepad1.x);

                //TRACAO
                driveMecanum.driveControl(Math.floor(gamepad1.left_stick_x * 10) / 10,
                        Math.floor(gamepad1.left_stick_y * 10) / 10,
                        Math.floor(gamepad1.right_stick_x * 10) / 10,
                        gamepad1.b);

                telemetry.addData("bl", driveMecanum.getBL().getCurrentPosition());
                telemetry.addData("motorBraco", braco.getMotorBraco().getCurrentPosition());
                telemetry.update();

                if (gamepad2.dpad_down) braco.bandeja.travaAutonomo();
                if (gamepad2.dpad_right) braco.bandeja.setAutoZero();

        }
    }
}