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
    //block fields
    private int row = 10; //default
    private int col = 10; //default
    private int blockWidth;
    private int blockHeight;
    private ArrayList<GridBlock> blockList;
    private ArrayList<GridBlock> detectedBlocks;
    private ArrayList<GridBlock> undetectedBlocks;
    //blue scalars
    private Scalar blueLowHSV = new Scalar(90, 140, 20);
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
    public DetectionProcessor(int r, int c, DetectionColor color, Telemetry t) {
        row = r;
        col = c;
        detectionColor = color;
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
                blockList.add(new GridBlock(j * blockWidth, i * blockHeight, blockWidth, blockHeight, j + i * row, detectionColor));
            }
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //convert frame to HSV
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        detectedBlocks = new ArrayList<GridBlock>();
        undetectedBlocks = new ArrayList<GridBlock>();
        //process all the blocks and sort them into their respective lists
        for(GridBlock gb : blockList) {
            gb.processBlock(frame);
            if(gb.isDetected()) {
                detectedBlocks.add(gb);
            } else {
                undetectedBlocks.add(gb);
            }
        }

        //threshold camera stream for viewer
        if(detectionColor == DetectionColor.BLUE) {
            Core.inRange(frame, blueLowHSV, blueHighHSV, frame);
        } else if(detectionColor == DetectionColor.RED) {
            Core.inRange(frame, redLowHSV, redHighHSV, red1);
            Core.inRange(frame, redLowHSV2, redHighHSV2, red2);
            Core.bitwise_or(red1, red2, frame);
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        //draw undetected blocks first
        for(GridBlock gb : undetectedBlocks) {
            rectPaint.setColor(gb.getEdgeColor());
            canvas.drawRect(gb.constructCanvasElem(scaleBmpPxToCanvasPx), rectPaint);
        }
        //draw detected blocks
        for(GridBlock gb : detectedBlocks) {
            rectPaint.setColor(gb.getEdgeColor());
            canvas.drawRect(gb.constructCanvasElem(scaleBmpPxToCanvasPx), rectPaint);
        }
    }

    //enum for detection color
    public enum DetectionColor {
        RED,
        BLUE
    }
}
