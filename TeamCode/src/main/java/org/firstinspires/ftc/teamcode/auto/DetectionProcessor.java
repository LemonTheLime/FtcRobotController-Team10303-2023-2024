package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/* DetectionProcessor
 * Computer vision processor that uses grids to detect red or blue objects
 */
public class DetectionProcessor implements VisionProcessor {

    //FIELDS
    //detection color
    private DetectionColor detectionColor = null;
    private RelativePos relativePos = null;
    private boolean active = false;
    //block fields
    private int row = 10; //default
    private int col = 10; //default
    private int blockWidth;
    private int blockHeight;
    private ArrayList<GridBlock> blockList;
    //blue scalars
    private Scalar blueLowHSV = new Scalar(90, 50, 20);
    private Scalar blueHighHSV = new Scalar(140, 255, 255);
    //red scalars (2 ends of spectrum)
    private Scalar redLowHSV = new Scalar(0, 20, 20);
    private Scalar redHighHSV = new Scalar(10, 255, 255);
    private Scalar redLowHSV2 = new Scalar(150, 20, 20);
    private Scalar redHighHSV2 = new Scalar(179, 255, 255);
    //red thresholding for camera stream
    private Mat red1 = new Mat();
    private Mat red2 = new Mat();
    //telemetry
    Telemetry telemetry = null;

    //constructor
    public DetectionProcessor(int r, int c, DetectionColor color, RelativePos pos, Telemetry t) {
        row = r;
        col = c;
        detectionColor = color;
        relativePos = pos;
        telemetry = t;
        telemetry.addData("detectionColor", detectionColor);
        telemetry.update();
    }


    public void init(int width, int height, CameraCalibration calibration) {
        blockList = new ArrayList<GridBlock>();
        blockWidth = (int)Math.floor(width / col);
        blockHeight = (int)Math.floor(height / row);
        for(int i = 0; i < row; i++) {
            for(int j = 0; j < col; j++) {
                blockList.add(new GridBlock(j * blockWidth, i * blockHeight, blockWidth, blockHeight, j + i * col, detectionColor));
            }
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //convert frame to HSV
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        //process all the blocks
        for(GridBlock gb : blockList) {
            gb.processBlock(frame);
        }

        //threshold camera stream for viewer
        if(detectionColor == DetectionColor.BLUE) {
            Core.inRange(frame, blueLowHSV, blueHighHSV, frame);
        } else if(detectionColor == DetectionColor.RED) {
            //combine thresholds
            Core.inRange(frame, redLowHSV, redHighHSV, red1);
            Core.inRange(frame, redLowHSV2, redHighHSV2, red2);
            Core.bitwise_or(red1, red2, frame);
            //cleanup
            red1.release();
            red2.release();
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 2);

        //draw undetected blocks first
        for(GridBlock gb : blockList) {
            if(!gb.isDetected()) {
                rectPaint.setColor(gb.getEdgeColor());
                canvas.drawRect(gb.constructCanvasElem(scaleBmpPxToCanvasPx), rectPaint);
            }
        }
        //draw detected blocks second
        for(GridBlock gb : blockList) {
            if(gb.isDetected()) {
                rectPaint.setColor(gb.getEdgeColor());
                canvas.drawRect(gb.constructCanvasElem(scaleBmpPxToCanvasPx), rectPaint);
            }
        }

        active = true;
    }

    //scans a certain region of blocks
    private boolean scanRegion(int x, int y, int width, int height, double threshold_percent) {
        int sum = 0;
        for(int i = y; i < y + height; i++) {
            for(int j = x; j < x + width; j++) {
                //regionalize blocks
                blockList.get(j + i * col).setRegionalized(true);

                //add to sum if detected
                if(blockList.get(j + i * col).isDetected()) {
                    sum++;
                };
            }
        }
        //calculate threshold average
        double thresh_avg = ((double)sum) / (width * height);

        //telemetry
        telemetry.addData("Threshold Average", thresh_avg);
        telemetry.update();

        if(thresh_avg > threshold_percent) {
            return true;
        }
        return false;
    }

    //scans 1000 times (or maybe more) for the spikemark location
    public int scanForSpikeMark() {
        if(relativePos == RelativePos.RIGHT) {
            //scan right spikemark
            boolean check = scanRegion(17, 14, 6, 5, 0.5);
            if(check) {
                return 3;
            } else {
                //scan middle spikemark
                check = scanRegion(4, 13, 5, 5, 0.5);
                if(check) {
                    return 2;
                } else {
                    //none detected so must be left
                    return 1;
                }
            }
        } else {
            //left side
            //scan right spikemark
            boolean check = scanRegion(26, 14, 4, 5, 0.5);
            if(check) {
                return 3;
            } else {
                //scan middle spikemark
                check = scanRegion(12, 13, 5, 6, 0.5);
                if(check) {
                    return 2;
                } else {
                    //must be left spike
                    return 1;
                }
            }

        }
    }

    //enum for detection color
    public enum DetectionColor {
        RED,
        BLUE
    }

    //enum for relative position
    public enum RelativePos {
        RIGHT,
        LEFT
    }

    //get activity of processor
    public boolean getActivity() {
        return active;
    }
}
