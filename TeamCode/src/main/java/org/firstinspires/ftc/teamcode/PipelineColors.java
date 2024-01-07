package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


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

        return mat;
    }

}