package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="teste", group="Linear OpMode")
//@Disabled
public class Teste extends LinearOpMode {

    OpenCvWebcam webcam = null; //declarar a webcam
    String linha;

    @Override
    public void runOpMode() {

        WebcamName webcamName   = hardwareMap.get(WebcamName.class, "Webcam 1"); //mapeamento da webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                                  hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); //declaração do id da webcam

        webcam.setPipeline(new Pipeline()); //definindo o pipeline

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //definindo o tamanho de tela da sua câmera
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error while opening camera: ", errorCode);
            }
        });

        waitForStart();
    }

    class Pipeline extends OpenCvPipeline{  //contrução do piperline
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat mat = new Mat();
        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
        //função proscess frame, irá detectar a localização geral de um objeto
        public Mat processFrame(Mat input){
            // e dividir o nosso espaço de vizualização em 3 quadrantes
            //parametros de altura e largura de cada quadrante
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HLS); //converter o feedback da camera para YCbCr, deixa o desempenho melhor por otimiza-lo
            telemetry.addLine("Pipeline rodando");

            Rect leftRect = new Rect(1, 1, 639, 1079);
            Rect rightRect = new Rect(1280, 1, 639, 1079);
            Rect midRect = new Rect(640, 1, 639, 1079);

            input.copyTo(mat);
            Imgproc.rectangle(mat, leftRect, rectColor, 2);
            Imgproc.rectangle(mat, rightRect, rectColor, 2); //esse código permite voce vizualizar os
            //quadrantes na sua driver hub, muito util para testes e verificação
            Imgproc.rectangle(mat, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];  //linhas de escaneamento
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            telemetry.addData("Esquerda", leftavgfin);
            telemetry.addData("Meio", midavgfin);
            telemetry.addData("Direita", rightavgfin);
            telemetry.update();

            if (leftavgfin > rightavgfin && leftavgfin > midavgfin){ // linhas para printar em qual quadrante o
                //objeto está sendo identificado
                telemetry.addLine("Esquerda");
                linha = "Esquerda";
            } else if (rightavgfin > midavgfin){
                telemetry.addLine("Direita");
                linha = "Direita";
            } else{
                telemetry.addLine("Meio");
                linha = "Meio";
            }

            return(mat);
        }
    }

}