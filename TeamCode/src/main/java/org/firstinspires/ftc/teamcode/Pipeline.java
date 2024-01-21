package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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


public class Pipeline extends OpenCvPipeline {
    private Mat mat, temp, c;

    private Scalar lower, upper;
    private Mat result = null;
    private int maxValIdx, ValX;
    private double contourArea, x, y, w, h, pos, maxVal, storage;
    public Size size;
//    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    public ElementLoc customElementLocation = ElementLoc.NOT_FOUND;

    public Pipeline() {
        mat = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {

        size = new Size(3,3);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HLS);


        Scalar lower = new Scalar(100, 100, 80);
        Scalar upper = new Scalar(150, 200, 200);

        //Scalar lower = new Scalar(0, 0, 0);
        //Scalar upper = new Scalar(255, 255, 255);



        Core.inRange(mat, lower, upper, mat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));


        Imgproc.threshold(mat, mat, 20, 255, Imgproc.THRESH_BINARY);
        temp = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
        Imgproc.erode(mat, mat, kernel);
        Imgproc.dilate(mat, mat, kernel);
        Imgproc.dilate(mat, mat, kernel);
        //Imgproc.dilate(mat, mat, kernel);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mat, contours, temp, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        maxVal = 0;
        maxValIdx = -1;

        if (contours.size() > 0) {

            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {

                contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (contourArea > 0 && contourArea > maxVal) {

                    maxVal = contourArea;
                    maxValIdx = contourIdx;

                } else if ((contourArea <= 0) && (maxValIdx > -1)) {

                    contours.remove(contourIdx);

                }
            }



            //Core.bitwise_and(input, input, result, mat);
            input.copyTo(mat);

            if (maxValIdx >= 0) {
                Imgproc.drawContours(mat, contours, maxValIdx, new Scalar(0, 0, 255), 5);

                Rect biggestRect = Imgproc.boundingRect(new MatOfPoint(contours.get(maxValIdx).toArray()));

                Point supDir = new Point(biggestRect.x, biggestRect.y);
                Point botEsc = new Point(biggestRect.x + biggestRect.width, biggestRect.y + biggestRect.height);

                storage = biggestRect.height * biggestRect.width;
                Imgproc.rectangle(mat, supDir, botEsc, new Scalar(0, 255, 0), 3);

                setLocation(biggestRect.x + biggestRect.width / 2);

            } else {
                customElementLocation = ElementLoc.NOT_FOUND;
            }

        }


        switch(getLocation()){

            case LEFT:  //LEFT ~ to define
                Imgproc.rectangle(mat, new Point(100, 190), new Point(100, 230), new Scalar(0, 255, 255), 3);
                break;

            case RIGHT:  //RIGHT ~ to define
                Imgproc.rectangle(mat, new Point(430, 190), new Point(430, 230), new Scalar(0, 255, 255), 3);
                break;

            case CENTER:  //CENTER ~ to define
                Imgproc.rectangle(mat, new Point(310, 190), new Point(310, 230), new Scalar(0, 255, 255), 3);
                break;

            case NOT_FOUND:
                default:

                Imgproc.line(mat, new Point(200, 180), new Point(400, 290), new Scalar(0, 255, 255), 3);

                Imgproc.line(mat, new Point(400, 180), new Point(200, 290), new Scalar(0, 255, 255), 3);
        }


        return mat;

    }
    private void setLocation(int valX) {

        this.ValX = ValX;
        if (valX < 100) {

            customElementLocation = ElementLoc.LEFT;

        } else if (valX > 180) {

            customElementLocation = ElementLoc.RIGHT;

        } else if (valX > 100 && valX < 180) {

            customElementLocation = ElementLoc.CENTER;

        } else {

            customElementLocation = ElementLoc.NOT_FOUND;

        }
    }

    public ElementLoc getLocation() {
        return customElementLocation;
    }


    public void setPos(ElementLoc pos){
        pos = customElementLocation;
    }

    public int getValX(){
        return ValX;
    }
    public int getMaxValIdx(){
        return maxValIdx;
    }
    public double getMaxVal(){
        return storage;
    }

}