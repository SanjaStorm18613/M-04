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
    Bandeja bandeja;
    Braco braco;
    BandejaTeste bandejaTeste;

    //Detector detector;
    

    private boolean armBlockUp = false, armBlockDown = false, bandejaBlock = false, bandejaBlockTotal = false, bandejaBlockRight = false,
                    bandejaBlockLeft = false, rollBandejaLeftBlock = false, rollBandejaRightBlock = false, FSUnlock = false,
                    sistemaLinearBlockDown = false, droneBlock = false, pitchBandejaUp = false, pitchBandejaDown = false,
                    bandejaBlockTrava = false, escalar = false;

    @Override
    public void runOpMode() {

        driveMecanum  = new DriveMecanum(this);
        sistemaLinear = new SistemaLinear(this);
        lancador      = new Lancador(this);
        coletor       = new Coletor(this);
        //bandeja       = new Bandeja(this);
        braco         = new Braco(this);
        bandejaTeste   = new BandejaTeste(this);
        //detector      = new Detector();

        //telemetry.addData("TeleOP" , "Inicializando");

        waitForStart();

        while(opModeIsActive()) {

            // DroneLauncher
            if (gamepad1.y) lancador.lancarDrone();

            //COLETOR
            //coletor.repel(Math.round(-gamepad1.right_trigger * 10) / 10.0);
            //coletor.collect(Math.floor(gamepad1.left_trigger * 10) / 10.0);

            coletor.collectorControl(gamepad1.left_trigger, -gamepad1.right_trigger);
            telemetry.addData("leftTrigger", gamepad1.left_trigger);
            telemetry.addData("rightTrigger", gamepad1.right_trigger);

            //BRACO
            braco.pitch((int)gamepad2.left_trigger * 100, (int)gamepad2.right_trigger * 100);
            if(gamepad1.x){
                braco.escalar();
            }

            braco.block(gamepad2.x); //Trava o braco para que nao caia

            if(gamepad2.left_bumper){
                braco.travaPos(.6);
            }

/*
            armBlockUp = gamepad1.dpad_up;
            armBlockDown = gamepad1.dpad_down;

            if (gamepad1.a) {
                sistemaLinear.retrairSistemaTotal();
                bandeja.rollBandeja(0);
            }
*/
            //SistemaLinear
            sistemaLinear.movimentarSistema(gamepad1.left_bumper, gamepad1.right_bumper);

/*
            //Bandeja
            bandeja.pitchBandeja(braco.getMotorBraco().getCurrentPosition(), (Math.floor(gamepad2.right_trigger * 100) / 100));

            if (gamepad2.a && !bandejaBlockRight){
                bandeja.rollBandeja(1);
                bandeja.destravarBandeja();

            } else if (gamepad2.b && !bandejaBlockLeft){
                bandeja.rollBandeja(-1);
                bandeja.destravarBandeja();
            }
            bandejaBlockRight = gamepad2.a;
            bandejaBlockLeft  = gamepad2.b;

            if (gamepad2.left_bumper && !bandejaBlock){
                bandeja.destravarBandeja();
            } else if (gamepad2.right_bumper && !bandejaBlockTotal){
                bandeja.destravarBandejaTotal();
                bandeja.rollBandeja(1);
            }
            bandejaBlock      = gamepad2.left_bumper;
            bandejaBlockTotal = gamepad2.right_bumper;

            if (gamepad2.dpad_right && !rollBandejaRightBlock){
                bandeja.rollBandeja(1);
            } else if (gamepad2.dpad_left && !rollBandejaLeftBlock){
                bandeja.rollBandeja(-1);
            }
            rollBandejaRightBlock = gamepad2.dpad_right;
            rollBandejaLeftBlock  = gamepad2.dpad_left;
            */


            /*if (gamepad1.a && !bandejaBlock){
                bandeja.destravarBandejaTotal();
            }
            if (gamepad1.b){
                bandeja.travarBandeja();
            }

            if (braco.getMotorBraco().getCurrentPosition() > 500) {

                bandeja.pitchBandeja(gamepad2.dpad_up, gamepad2.dpad_down);

            } else {

               bandeja.getServoPitch().setPosition(.2);

            }

            if (gamepad1.dpad_left && !rollBandejaLeftBlock) {

                bandeja.rollBandeja(.3);
            }
            if (gamepad1.dpad_right && !rollBandejaRightBlock) {

                bandeja.rollBandeja(.7);
            }*/

            //BANDEJA TEST
            if((!rollBandejaLeftBlock || !rollBandejaRightBlock) && (gamepad2.dpad_left || gamepad2.dpad_right)) {
                bandejaTeste.rollBandeja(gamepad2.dpad_left, gamepad2.dpad_right); //Roll
            }
            if(gamepad2.a && !bandejaBlock) {
                bandejaTeste.destravarBandeja(); //Destravar
            }
            if(gamepad2.b && !bandejaBlockTotal){
                bandejaTeste.destravarBandejaTotal(); //Destravar
            }
            if(gamepad2.y && !bandejaBlockTrava){
                bandejaTeste.travarBandeja(); //Travar

            }
            if((!pitchBandejaUp || !pitchBandejaDown) && (gamepad2.dpad_up || gamepad2.dpad_down)) {
                bandejaTeste.pitchBandeja(gamepad2.dpad_up, gamepad2.dpad_down); //Pitch
            }
            //bandejaTeste.rollBandeja(gamepad2.dpad_left, gamepad2.dpad_right);

            rollBandejaRightBlock = gamepad2.dpad_right;
            rollBandejaLeftBlock = gamepad2.dpad_left;

            pitchBandejaDown = gamepad2.dpad_down;
            pitchBandejaUp = gamepad2.dpad_up;

            bandejaBlock = gamepad2.a;
            bandejaBlockTrava = gamepad2.y;
            bandejaBlockTotal = gamepad2.b;

            //telemetry.addData("bandeja", bandeja.getServoBandeja());

            //Tração
            driveMecanum.periodic(Math.floor(gamepad1.left_stick_x * 10) / 10,
                                  Math.floor(gamepad1.left_stick_y * 10) / 10,
                                Math.floor(gamepad1.right_stick_x * 10) / 10);
            //telemetry.addData("odomX", driveMecanum.getOdomX().getCurrentPosition());
            telemetry.addData("bl", driveMecanum.getBL().getCurrentPosition());
            telemetry.addData("odomY", driveMecanum.getOdomY().getCurrentPosition());
            telemetry.addData("motorBraco", braco.getMotorBraco().getCurrentPosition());
            telemetry.update();

            //telemetry.addData("X", driveMecanum.getX());
            //telemetry.update();

        }
    }
}