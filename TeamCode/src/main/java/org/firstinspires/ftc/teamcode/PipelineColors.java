package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class SquareLocationDetectorOpenCV extends OpenCvPipeline {
    Mat mat;

    public SquareLocationDetectorOpenCV() {
        mat = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HLS);

        Scalar lower = new Scalar(30, 10, 150);
        Scalar upper = new Scalar (100, 80, 255);

        Core.inRange(mat, lower, upper, mat);


        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Mat temp = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2));
        Imgproc.findContours(mat, contours, temp, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);






        return mat;

    }

}