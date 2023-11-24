package org.firstinspires.ftc.teamcode.cvtest;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;


public class ThresholdProcessor implements VisionProcessor {

    public Rect rect;

    //20 is current min threshold

    //red threshold
    private Scalar redLowHSV = new Scalar(0, 20, 20);
    private Scalar redHighHSV = new Scalar(10, 255, 255);

    //blue threshold
    private Scalar blueLowHSV = new Scalar(90, 140, 20);
    private Scalar blueHighHSV = new Scalar(140, 255, 255);

    //red threshold (other end of spectrum)
    private Scalar redLowHSV2 = new Scalar(150, 20, 20);
    private Scalar redHighHSV2 = new Scalar(179, 255, 255);

    //color mode, 0 = red, 1 = blue, 3 = red2 (other end of spectrum)
    private int mode = 0;

    //combined red
    private Mat red1 = new Mat();
    private Mat red2 = new Mat();


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        rect = new Rect(0, 0, width, height);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);


        //threshold
        if(mode == 1) {
            Core.inRange(frame, redLowHSV, redHighHSV, frame);

        } else if(mode == 2) {
            Core.inRange(frame, blueLowHSV, blueHighHSV, frame);
        } else if(mode == 3) {
            Core.inRange(frame, redLowHSV2, redHighHSV2, frame);
        } else {
            //combine both reds
            Core.inRange(frame, redLowHSV, redHighHSV, red1);
            Core.inRange(frame, redLowHSV2, redHighHSV2, red2);
            Core.bitwise_or(red1, red2, frame);

        }

        return null; // No context object
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }

    //create android rectangle object from opencv rectangle
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    //set threshold to red or blue mode
    public void setColorMode(ElementColor color) {
        switch (color) {
            case RED:
                mode = 1;
                break;
            case BLUE:
                mode = 2;
                break;
            case RED2:
                mode = 3;
                break;
            case NONE:
                mode = 0;
        }

    }

    public enum ElementColor {
        RED,
        BLUE,
        RED2,
        NONE
    }



}