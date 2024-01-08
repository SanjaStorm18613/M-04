package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class Detector extends OpenCvPipeline {
    private Mat mat, temp, c;
    private Mat result = null;
    private int maxValIdx;
    private double contourArea, x, y, w, h, pos;
    public double[] size;
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private ElementLoc customElementLocation = ElementLoc.NOT_FOUND;

    public Detector() {
        mat = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {


        size = new double[]{x, y, w, h};
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HLS);

        Scalar lower = new Scalar(87, 77, 46);
        Scalar upper = new Scalar(124, 80, 255);

        Core.inRange(mat, lower, upper, mat);

        Imgproc.threshold(mat, mat, 20, 255, Imgproc.THRESH_BINARY);

        temp = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
        Imgproc.findContours(mat, contours, temp, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        if (result != null) {
            result.release();
        } else {
            result = new Mat();
        }

        if (contours.size() > 0) {

            double maxVal = 0;
            int maxValIdx = -1;

            Imgproc.drawContours(result, contours, -1, new Scalar(0, 0, 255), 5);

            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {

                contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (contourArea > 500 && contourArea > maxVal) {

                    maxVal = contourArea;
                    maxValIdx = contourIdx;

                } else if ((contourArea <= 500) && (maxValIdx > -1)) {

                    contours.remove(contourIdx);

                }

            }


        }

        Core.bitwise_and(input, input, result, mat);
        mat.release();

        if(maxValIdx >= 0){

            Rect biggestRect = Imgproc.boundingRect(new MatOfPoint(contours.get(maxValIdx).toArray()));


            Point supDir = new Point(biggestRect.x, biggestRect.y);
            Point botEsc = new Point(biggestRect.x + biggestRect.width, biggestRect.y + biggestRect.height);


            Imgproc.rectangle(result, supDir, botEsc, new Scalar(0, 255, 0), 3);


            //pos = x + w/2, y + h/2
            setLocation(biggestRect.x + biggestRect.width / 2);

        }
        else {
            this.customElementLocation = ElementLoc.NOT_FOUND;
        }

        switch(getLocation()){

            case LEFT:  //LEFT ~176
                Imgproc.line(result, new Point(100, 190), new Point(100, 230), new Scalar(0, 255, 255), 3);
                break;

            case RIGHT:  //RIGHT ~515
                Imgproc.line(result, new Point(430, 190), new Point(430, 230), new Scalar(0, 255, 255), 3);
                break;

            case CENTER:  //CENTER ~327
                Imgproc.line(result, new Point(312, 190), new Point(312, 230), new Scalar(0, 255, 255), 3);
                break;

            case NOT_FOUND:
                default:
                Imgproc.line(result, new Point(200, 180), new Point(400, 290), new Scalar(0, 255, 255), 3);

                Imgproc.line(result, new Point(400, 180), new Point(200, 290), new Scalar(0, 255, 255), 3);
        }
        telemetry.addData("Location", customElementLocation);
        telemetry.update();

        return result;
    }
    private void setLocation(int valX) {

        if (valX < 240)  {
            this.customElementLocation = ElementLoc.LEFT;
        }
        else if (valX > 380) {
            this.customElementLocation = ElementLoc.RIGHT;
        }
        else {
            this.customElementLocation = ElementLoc.CENTER;
        }
    }



    public ElementLoc getLocation() {
        return customElementLocation;
    }

    public enum ElementLoc {
        LEFT, CENTER, RIGHT, NOT_FOUND;
    }

}