package org.firstinspires.ftc.teamcode.test.cvtest;

/* Detects red and blue elements and uses a grid system to track what areas detect them
 */

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class GridVisionProcessor implements VisionProcessor {

    //FIELDS
    private int row = 10;
    private int col = 10;
    private int blockWidth;
    private int blockHeight;
    private ArrayList<GridBlock> blockList;

    private Scalar blueLowHSV = new Scalar(90, 140, 20);
    private Scalar blueHighHSV = new Scalar(140, 255, 255);

    //create the blocks
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        blockList = new ArrayList<GridBlock>();
        blockWidth = (int)Math.floor(width / col);
        blockHeight = (int)Math.floor(height / row);
        for(int i = 0; i < row; i++) {
            for(int j = 0; j < col; j++) {
                blockList.add(new GridBlock(j * blockWidth, i * blockHeight, blockWidth, blockHeight));
            }
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        for(GridBlock gb : blockList) {
            gb.processBlock(frame);
        }

        Core.inRange(frame, blueLowHSV, blueHighHSV, frame);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        for(GridBlock gb : blockList) {
            rectPaint.setColor(gb.getEdgeColor());
            canvas.drawRect(gb.constructCanvasElem(scaleBmpPxToCanvasPx), rectPaint);
        }
    }
}
