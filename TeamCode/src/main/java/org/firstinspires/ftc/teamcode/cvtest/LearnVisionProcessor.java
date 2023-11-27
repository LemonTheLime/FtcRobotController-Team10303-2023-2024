package org.firstinspires.ftc.teamcode.cvtest;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class LearnVisionProcessor implements VisionProcessor {



    public Rect rectLeft, rectMiddle, rectRight;
    Selected selection = Selected.NONE;
    String rectangleSelected = "none";

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    private int WIDTH;
    private int HEIGHT;

    private double satRectLeft, satRectMiddle, satRectRight;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        WIDTH = width;
        HEIGHT = height;
        rectLeft = new Rect(0, 0, WIDTH / 3, HEIGHT / 2);
        rectMiddle = new Rect(WIDTH / 3, 0, WIDTH / 3, HEIGHT / 2);
        rectRight = new Rect(2 * WIDTH / 3, 0, WIDTH / 3, HEIGHT / 2);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        satRectRight = getAvgSaturation(hsvMat, rectRight);


        if((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            selection = Selected.LEFT;
            rectangleSelected = "left";
        } else if((satRectMiddle > satRectLeft) && (satRectLeft > satRectRight)) {
            selection = Selected.MIDDLE;
            rectangleSelected = "middle";
        } else if((satRectRight > satRectLeft) && (satRectRight > satRectMiddle)) {
            selection = Selected.RIGHT;
            rectangleSelected = "right";
        } else {
            selection = Selected.NONE;
            rectangleSelected = "none";
        }
        return null;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        switch (rectangleSelected) {
            case "left":
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case "middle":
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case "right":
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case "none":
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }


    }

    public Selected getSelection() {
        return selection;
    }

    public double getAvgReading(String pos) {
        if(pos.equals("left")) {
            return satRectLeft;
        } else if(pos.equals("middle")) {
            return satRectMiddle;
        }
        return satRectRight;
    }

    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}
