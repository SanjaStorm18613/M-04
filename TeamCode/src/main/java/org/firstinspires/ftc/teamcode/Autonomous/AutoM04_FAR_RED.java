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

package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Braco;
import org.firstinspires.ftc.teamcode.Subsystems.Coletor;
import org.firstinspires.ftc.teamcode.Subsystems.DriveMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Lancador;
import org.firstinspires.ftc.teamcode.Subsystems.SistemaLinear;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name= "AutoM04_FAR_RED", group = "Robot")
//@Disabled
public class AutoM04_FAR_RED extends LinearOpMode {

    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Lancador lancaDrone;
    Coletor coletor;
    Braco braco;
    ElapsedTime timer;
    ElementLoc loc;
    private AprilTagProcessor aprilTag;
    private Pipeline_Vermelho pipelineVermelho;
    private VisionPortal visionPortal;

    int step = 0;


    @Override
    public void runOpMode(){
        telemetry.addData("Inicializando auto", " ");

        driveMecanum = new DriveMecanum(this);

        sistemaLinear = new SistemaLinear(this);

        lancaDrone = new Lancador(this);

        braco = new Braco(this);

        timer = new ElapsedTime();

        initCamera();

        timer.reset();
        telemetry.update();
        while(!isStarted() && !isStopRequested()){
            if (pipelineVermelho.getLocation() != ElementLoc.NOT_FOUND) {
                loc = pipelineVermelho.getLocation();
                visionPortal.setProcessorEnabled(pipelineVermelho,false);
            } else if(pipelineVermelho.getLocation() == ElementLoc.NOT_FOUND){
                loc = ElementLoc.CENTER;
                visionPortal.setProcessorEnabled(pipelineVermelho,false);
            }
            telemetry.addData("camera", loc);
            telemetry.update();
        }
        while (opModeIsActive()) {
            switch(loc){
                case CENTER:
                    if(step == 0) driveMecanum.moveForwardAuto(-0.7, 1300);

                    if(driveMecanum.getBL().getCurrentPosition() >= 1290 && step == 0) {
                        resetEnc_step();
                    }

                    if(step == 1) {
                        braco.bandeja.travaAutonomo();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }

                    if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 1) {
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                    }
                    break;
                case LEFT:
                    if(step == 0) driveMecanum.moveForwardAuto(-0.7, 1250);

                    if(driveMecanum.getBL().getCurrentPosition() >= 1240 && step == 0) {
                        resetEnc_step();
                    }
                    if(step == 1) driveMecanum.turn(.6, 900);

                    if(driveMecanum.getBL().getCurrentPosition() >= 890 && step == 1){
                        resetEnc_step();
                    }

                    if(step == 2){
                        braco.bandeja.travaAutonomo();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }
                    if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 2) {
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                    }

                    break;
                case RIGHT:
                    if(step == 0) driveMecanum.moveForwardAuto(-0.7, 1250);

                    if(driveMecanum.getBL().getCurrentPosition() > 1240 && step == 0) {
                        resetEnc_step();
                    }

                    if(step == 1) driveMecanum.turn(.6, -950);

                    if(driveMecanum.getBL().getCurrentPosition() < -940 && step == 1){
                        resetEnc_step();
                    }

                    if (step == 2){
                        driveMecanum.moveForwardAuto(.4, 50);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 45 && step == 2){
                        resetEnc_step();
                    }
                    if(step == 3){
                        braco.bandeja.travaAutonomo();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }
                    if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 3) {
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                    }

                    break;
                default:
                    break;
            }
        }
    }
    public void resetEnc_step(){
        driveMecanum.resetEnc();
        step++;
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void initCamera() {

        // Create the AprilTag processor.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        pipelineVermelho = new Pipeline_Vermelho();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(pipelineVermelho);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(pipelineVermelho, true);

    }   // end method initAprilTag()
}
