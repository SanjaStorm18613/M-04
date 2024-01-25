package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.channels.Pipe;

public class VisionControl {

    private OpenCvWebcam webcam;
    private LinearOpMode opMode;

    private Pipeline_Vermelho pipelineVermelho;
    private Pipeline_Azul pipelineAzul;

    public VisionControl(LinearOpMode opMode) {

        //WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                                  opMode.hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get
                        (WebcamName.class,  "Webcam 1"), cameraMonitorViewId);

        initDetectionElement();
        pipelineAzul = new Pipeline_Azul();
        pipelineVermelho = new Pipeline_Vermelho();
        webcam.setPipeline(pipelineVermelho);
        webcam.setPipeline(pipelineAzul);
    }

    public void initDetectionElement() {

        webcam.setMillisecondsPermissionTimeout(2500);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //opMode.telemetry.addData("Error while opening camera: ", errorCode);
            }
        });
    }

    public void setPipeline(OpenCvPipeline detector) {
        webcam.setPipeline(detector);
    }

    public void stopDetection(){
        webcam.pauseViewport();
        //webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
    public void stopViewport() {
        webcam.pauseViewport();
    }

    public Pipeline_Vermelho getPipelineVermelho(){
        return pipelineVermelho;
    }
    public Pipeline_Azul getPipelineAzul(){
        return pipelineAzul;
    }
    public boolean getDetected(){
        boolean getDetected = false;
        if(pipelineVermelho.getLocation() == ElementLoc.LEFT || pipelineVermelho.getLocation() == ElementLoc.RIGHT || pipelineVermelho.getLocation() == ElementLoc.CENTER){
            getDetected = true;
        }
        else{
            getDetected = false;
        }
        return getDetected;
    }
}
