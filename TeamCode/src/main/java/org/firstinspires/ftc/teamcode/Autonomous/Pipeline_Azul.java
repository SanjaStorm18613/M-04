package org.firstinspires.ftc.teamcode.Autonomous;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;


public class Pipeline_Azul implements VisionProcessor {
    private final Paint green;
    private Mat mat, temp;
    private Scalar lower, upper;
    private int maxValIdx;
    private double contourArea, maxVal;
    public Size size;
    private ArrayList<MatOfPoint> contours;
    private Point p;
    public ElementLoc customElementLocation = ElementLoc.NOT_FOUND;

    public Pipeline_Azul() {
        mat = new Mat();

        green = new Paint();
        green.setColor(Color.rgb(0,255,0));

        p = new Point(-1,-1);
    }

    private void setLocation(int valX) {
        if (valX < 400) {
            customElementLocation = ElementLoc.LEFT;
        } else if (valX > 1000) {
            customElementLocation = ElementLoc.RIGHT;
        } else if (valX > 400 && valX < 1000) {
            customElementLocation = ElementLoc.CENTER;
        } else {
            customElementLocation = ElementLoc.NOT_FOUND;
        }

    }

    public ElementLoc getLocation() {
        return customElementLocation;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lower = new Scalar (0, 50, 60);
        upper = new Scalar (30, 150, 180);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        size = new Size(3,3);

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_BGR2HLS);

        Core.inRange(mat, lower, upper, mat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

        Imgproc.threshold(mat, mat, 20, 255, Imgproc.THRESH_BINARY);
        temp = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2));
        Imgproc.erode(mat, mat, kernel, p, 4);
        Imgproc.dilate(mat, mat, kernel, p ,4);

        contours = new ArrayList<>();

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
            frame.copyTo(mat);
        }
        return mat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (contours.size() > 0) {
            Rect bRect = Imgproc.boundingRect(new MatOfPoint(contours.get(maxValIdx).toArray()));

            canvas.drawRect(bRect.x, bRect.y,bRect.x + bRect.width, bRect.y + bRect.height, green);

            setLocation(bRect.x + bRect.width / 2);
        } else {
            customElementLocation = ElementLoc.NOT_FOUND;
        }
    }
}