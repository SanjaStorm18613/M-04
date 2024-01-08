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


@TeleOp(name="TeleopM04", group="Linear OpMode")

public class TeleopM04 extends LinearOpMode {
    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Lancador lancador;
    Coletor coletor;
    Bandeja bandeja;
    Braco braco;
    

    private boolean armBlockUp = false, armBlockDown = false, bandejaBlock = false, bandejaBlockTotal = false, bandejaBlockRight = false,
                    bandejaBlockLeft = false, rollBandejaLeftBlock = false, FSUnlock = false, sistemaLinearBlockDown = false,
                    droneBlock = false;


    @Override
    public void runOpMode() {

        driveMecanum  = new DriveMecanum(this);
        sistemaLinear = new SistemaLinear(this);
        lancador    = new Lancador(this);
        //coletor       = new Coletor(this);
        bandeja       = new Bandeja(this);
        braco         = new Braco(this);

        //telemetry.addData("TeleOP" , "Inicializando");

        waitForStart();

        while(opModeIsActive()) {

            // DroneLauncher
            if(gamepad1.y){
                lancador.lancarDrone();
            }
            //telemetry.addData("Drone", lancador.getServoDrone().getPosition());

            //Collector
            //coletor.collect(Math.floor(gamepad1.right_trigger * 10) / 10.);
            //coletor.repel(Math.floor(gamepad1.left_trigger * 10) / 10.);

            //Arm
            //if (gamepad1.dpad_up && !armBlockUp) {
              //  braco.BracoUp();
            //} else if (gamepad1.dpad_down && !armBlockDown) {
              //  braco.BracoDown(.5);
            //}

            //if(gamepad1.left_trigger > .1){ braco.PitchBraco(Constants.Braco.stage1, gamepad1.left_trigger); }
            //if(gamepad1.right_trigger > .1){ braco.PitchBraco(Constants.Braco.stage0, gamepad1.right_trigger); }

            braco.pitch((int) gamepad1.right_trigger * 100,
                      (int) gamepad1.left_trigger * 100);

            /*braco.block(gamepad1.dpad_right ? .5 : 0);

            braco.travaPos(gamepad1.dpad_left ? 1 : 0);


            //telemetry.addData("motor", braco.getMotorBraco().getCurrentPosition());
            //telemetry.addData("rightTrigger", gamepad1.right_trigger);
            //telemetry.addData("leftTrigger", gamepad1.left_trigger);

/*
            armBlockUp = gamepad1.dpad_up;
            armBlockDown = gamepad1.dpad_down;

            if (gamepad1.a) {
                sistemaLinear.retrairSistemaTotal();
                bandeja.rollBandeja(0);
            }
*/
            //SistemaLinear
            sistemaLinear.movimentarSistema(gamepad1.right_bumper, gamepad1.left_bumper);

            //if (gamepad1.b && !sistemaLinearBlockDown){
                //sistemaLinear.retrairSistemaTotal(.6, Constants.SistemaLinear.stage0);
            //}
            //sistemaLinearBlockDown = gamepad1.b;
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


            if(gamepad1.a && bandejaBlock){
                bandeja.destravarBandeja();
            }
            if(gamepad1.b){
                bandeja.travarBandeja();
            }

            if(gamepad1.dpad_up){
                bandeja.pitchBandeja();
            }
            if(gamepad1.dpad_down){
                bandeja.bandejaV();
            }

            if(gamepad1.dpad_left && rollBandejaLeftBlock){
                bandeja.rollBandeja(1);
            }

            rollBandejaLeftBlock = !gamepad1.dpad_left;
            bandejaBlock = !gamepad1.a;

            //telemetry.addData("bandeja", bandeja.getServoBandeja());

            //Tração
            driveMecanum.periodic(Math.floor(gamepad1.left_stick_x * 10) / 10,
                                  Math.floor(gamepad1.left_stick_y * 10) / 10,
                                Math.floor(gamepad1.right_stick_x * 10) / 10);

            //telemetry.addData("X", driveMecanum.getX());
            //telemetry.update();

        }
    }
}