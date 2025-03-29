package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class propDetectionCV implements VisionProcessor {
    //We need to implement the camera in a way that it can see all three lines on the ground
    //and it should be like equally split into threes

    Rect Left_side;
    Rect Middle;
    Rect right_side;

    Mat HSV = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //all the rects should get calibrated acourding to the output of our camera obv
        //also depending on where the camera is we should zoom in so that we dont like detect anything thats not ours
        Left_side = new Rect(
                new Point(0, 0), new Point(0.33*width, height)
        );
        Middle = new Rect(
                new Point(0.33*width, 0), new Point(0.66*width, height)
        );
        right_side = new Rect(
                new Point(0.66*width, 0), new Point(width, height)
        );
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, HSV, 41);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}

