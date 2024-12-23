package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Color;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

/* Grid block units for the GridVisionProcessor
 */
public class GridBlock {

    //FIELDS
    //block fields
    private int id;
    private DetectionProcessor.DetectionColor detectionColor = null;
    private Rect openCVRect;
    //threshold fields
    private final double PERCENT_THRESHOLD = 0.10;
    private double currentThreshold;
    private Mat blockMat = new Mat(); //stores the mat object for the image inside this block
    private Mat threshMat = new Mat(); //stores the final threshed blockMat
    private Mat red1 = new Mat();
    private Mat red2 = new Mat();
    private boolean colorIsDetected = false;
    //android canvas fields
    private android.graphics.Rect canvasRect;
    private int edgeColor = Color.GREEN;
    private boolean regionalized = false;
    //blue scalars
    private Scalar blueLowHSV = new Scalar(90, 50, 20);
    private Scalar blueHighHSV = new Scalar(140, 255, 255);
    //red scalars (2 ends of spectrum)
    private Scalar redLowHSV = new Scalar(0, 20, 20);
    private Scalar redHighHSV = new Scalar(10, 255, 255);
    private Scalar redLowHSV2 = new Scalar(150, 20, 20);
    private Scalar redHighHSV2 = new Scalar(179, 255, 255);



    //constructor - takes cv rectangle attributes and detection color
    public GridBlock(int x, int y, int width, int height, int i, DetectionProcessor.DetectionColor color) {
        openCVRect = new Rect(x, y, width, height);
        id = i;
        detectionColor = color;
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
        //submat the frame
        blockMat = frame.submat(openCVRect);

        //threshold
        if(detectionColor == DetectionProcessor.DetectionColor.BLUE) {
            Core.inRange(blockMat, blueLowHSV, blueHighHSV, threshMat);
        } else if(detectionColor == DetectionProcessor.DetectionColor.RED) {
            //join both thresholds
            Core.inRange(blockMat, redLowHSV, redHighHSV, red1);
            Core.inRange(blockMat, redLowHSV2, redHighHSV2, red2);
            Core.bitwise_or(red1, red2, threshMat);
            //cleanup
            red1.release();
            red2.release();
        }

        //cleanup blockmat
        blockMat.release();

        //calculate threshold percentage
        currentThreshold = Core.sumElems(threshMat).val[0] / openCVRect.area() / 255;
        if(currentThreshold > PERCENT_THRESHOLD) {
            colorIsDetected = true;
            if(!regionalized) {
                edgeColor = Color.RED;
            }
        } else {
            colorIsDetected = false;
            if(!regionalized) {
                edgeColor = Color.GREEN;
            }
        }

        //cleanup threshmat
        threshMat.release();

    }

    //return the paint color
    public int getEdgeColor() {
        return edgeColor;
    }

    //return detected for not
    public boolean isDetected() {
        return colorIsDetected;
    }

    public void setRegionalized(boolean mode) {
        if(mode) {
            regionalized = true;
            edgeColor = Color.BLUE;
        } else {
            regionalized = false;
        }
    }

}
