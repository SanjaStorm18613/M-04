package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionControl {

    private OpenCvWebcam webcam;
    private LinearOpMode opMode;

    private Detector pipeline;

    public VisionControl(LinearOpMode opMode) {

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                                  opMode.hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get
                        (WebcamName.class,  "Webcam 1"), cameraMonitorViewId);

        initDetectionElement();
        pipeline = new Detector();
        setPipeline(pipeline);
    }

    public void initDetectionElement() {

        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("Error while opening camera: ", errorCode);
            }
        });
    }

    public void setPipeline(OpenCvPipeline detector) {
        webcam.setPipeline(detector);
    }

    public void stopDetection(){
        webcam.pauseViewport();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
    public void stopViewport() {
        webcam.pauseViewport();
    }
}
