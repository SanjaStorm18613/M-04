/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TeleopM04", group = "Linear OpMode")

public class TeleopM04 extends LinearOpMode {
    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Lancador lancador;
    Coletor coletor;
    Braco braco;

    //Detector detector;
    

    private boolean armBlockUp = false, armBlockDown = false, bandejaBlock = false, bandejaBlockTotal = false, bandejaBlockRight = false,
                    bandejaBlockLeft = false, rollBandejaLeftBlock = false, rollBandejaRightBlock = false, FSUnlock = false,
                    sistemaLinearBlockDown = false, droneBlock = false, pitchBandejaUp = false, pitchBandejaDown = false,
                    bandejaBlockTrava = false, escalar = false, inverterMotorElevador = false, inverterMotorBraco = false;

    @Override
    public void runOpMode() {

        driveMecanum  = new DriveMecanum(this);
        sistemaLinear = new SistemaLinear(this);
        lancador      = new Lancador(this);
        coletor       = new Coletor(this);
        braco         = new Braco(this);

        //telemetry.addData("TeleOP" , "Inicializando");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {


            // DroneLauncher
            if(gamepad1.y) lancador.lancarDrone();

            //COLETOR
            coletor.collectorControl(gamepad1.left_trigger, -gamepad1.right_trigger);

            //BRACO
            if(gamepad2.back && !inverterMotorBraco) braco.inverterMotor();
            braco.pitch((int)gamepad2.left_trigger * 100, (int)gamepad2.right_trigger * 100);

            if (gamepad2.left_bumper) braco.travaPos();
            braco.block(gamepad2.x);

            inverterMotorBraco = gamepad2.back;

            //SistemaLinear
            sistemaLinear.movimentarSistema(gamepad1.left_bumper, gamepad1.right_bumper);
            if(gamepad1.back && !inverterMotorElevador) sistemaLinear.inverterMotor();

            inverterMotorElevador = gamepad1.back;


            //BANDEJA TEST
            if((!rollBandejaLeftBlock || !rollBandejaRightBlock) && (gamepad2.dpad_left || gamepad2.dpad_right)) {
                braco.bandejaTeste.rollBandeja(gamepad2.dpad_left, gamepad2.dpad_right); //Roll
            }
            if(gamepad2.a && !bandejaBlock) {
                braco.bandejaTeste.destravarBandeja(); //Destravar
            }
            if(gamepad2.b && !bandejaBlockTotal){
                braco.bandejaTeste.destravarBandejaTotal(); //Destravar
            }
            if(gamepad2.y && !bandejaBlockTrava){
                braco.bandejaTeste.travarBandeja(); //Travar

            }
            if(braco.bandejaTeste.getServoTravaBandeja().getPosition() == 1){
                braco.bandejaTeste.getLampada().setPower(.6);
            } else if(braco.bandejaTeste.getServoTravaBandeja().getPosition() == 0) {
                braco.bandejaTeste.getLampada().setPower(0);
            }
            braco.bandejaTeste.pitchBandeja(gamepad1.x);

            rollBandejaRightBlock = gamepad2.dpad_right;
            rollBandejaLeftBlock = gamepad2.dpad_left;

            pitchBandejaDown = gamepad2.dpad_down;
            pitchBandejaUp = gamepad2.dpad_up;

            bandejaBlock = gamepad2.a;
            bandejaBlockTrava = gamepad2.y;
            bandejaBlockTotal = gamepad2.b;

            //Tração
            driveMecanum.periodic(Math.floor(gamepad1.left_stick_x * 10) / 10,
                                  Math.floor(gamepad1.left_stick_y * 10) / 10,
                                Math.floor(gamepad1.right_stick_x * 10) / 10);

            telemetry.addData("bl", driveMecanum.getBL().getCurrentPosition());
            telemetry.addData("odomY", braco.bandejaTeste.getLampada().getCurrentPosition());
            telemetry.addData("motorBraco", braco.getMotorBraco().getCurrentPosition());
            telemetry.update();


        }
    }
}