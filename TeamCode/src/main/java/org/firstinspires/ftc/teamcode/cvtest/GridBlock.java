package org.firstinspires.ftc.teamcode.cvtest;

import android.graphics.Color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

/* Grid block units for the GridVisionProcessor
 */
public class GridBlock {

    //FIELDS
    private int left, right, top, bottom;
    private int id;
    private Rect openCVRect;
    private android.graphics.Rect canvasRect;
    private final double PERCENT_THRESHOLD = 0.25;
    private double currentThreshold;
    private Mat blockMat = new Mat(); //stores the mat object for the image inside this block
    private Mat threshMat = new Mat(); //stores the threshed blockMat

    private int edgeColor = Color.GREEN;


    //color scalars
    private Scalar blueLowHSV = new Scalar(90, 140, 20);
    private Scalar blueHighHSV = new Scalar(140, 255, 255);



    //constructor - takes cv rectangle attributes
    public GridBlock(int x, int y, int width, int height) {
        openCVRect = new Rect(x, y, width, height);
    }

    //construct canvas element
    public android.graphics.Rect constructCanvasElem(float scaleBmpPxToCanvasPx) {
        makeGraphicsRect(openCVRect, scaleBmpPxToCanvasPx);
        return canvasRect;
    }

    //creates an android canvas rectangle
    //create android rectangle object from opencv rectangle
    private void makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        canvasRect = new android.graphics.Rect(left, top, right, bottom);
    }

    //sets the submat of the frame
    public void processBlock(Mat frame) {
        blockMat = frame.submat(openCVRect);
        Core.inRange(blockMat, blueLowHSV, blueHighHSV, threshMat);
        currentThreshold = Core.sumElems(threshMat).val[0] / openCVRect.area() / 255;
        if(currentThreshold > PERCENT_THRESHOLD) {
            edgeColor = Color.RED;
        } else {
            edgeColor = Color.GREEN;
        }
    }

    //return the paint color
    public int getEdgeColor() {
        return edgeColor;
    }

}
