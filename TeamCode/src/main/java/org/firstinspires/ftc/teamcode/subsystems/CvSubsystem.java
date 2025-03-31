package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// THIS IS VERY BASIC IT WILL GET IMPROVED ON
public class CvSubsystem extends OpenCvPipeline implements VisionProcessor {
    //We need to implement the camera in a way that it can see all three lines on the ground
    //and it should be like equally split into threes

    Rect Left_side;
    Rect Middle;
    Rect right_side;

    Mat HSV = new Mat();
    Mat low = new Mat();
    Mat high = new Mat();
    Mat range = new Mat();

    Scalar Upper_Upper;
    Scalar Upper_Lower;
    Scalar Lower_Upper;
    Scalar Lower_Lower;

    double perecentLeft;
    double percentRight;
    double perecentMiddle;

    static boolean isread;

    int min = 10;

    propdetect prediction = propdetect.error;

    //Make sure to set red before innit
    public static void setIsread(boolean x){isread = x;}

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //Change HSV values obv
        if(isread){
            Upper_Upper  = new Scalar(170, 50, 50);
            Upper_Lower  = new Scalar(180, 50, 50);
            Lower_Upper  = new Scalar(0, 50, 50);
            Lower_Lower  = new Scalar(20, 50, 50);
        }else{
            Upper_Upper  = new Scalar(130, 255, 255);
            Upper_Lower  = new Scalar(110, 50, 50);
            Lower_Upper  = new Scalar(130, 255, 255);
            Lower_Lower  = new Scalar(110, 50, 50);
        }
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
        Core.inRange(HSV, Upper_Upper,Upper_Lower, high);
        Core.inRange(HSV, Lower_Upper,Lower_Lower, low);
        Core.bitwise_or(low, high, range);

        perecentLeft = (Core.sumElems(range.submat(Left_side)).val[0] / Left_side.area())  *100;
        percentRight = (Core.sumElems(range.submat(right_side)).val[0] /  right_side.area())  *100;
        perecentMiddle = (Core.sumElems(range.submat(Middle)).val[0] /  Middle.area()) *100;
        if(perecentLeft > perecentMiddle && perecentLeft > percentRight && perecentLeft > min){
            prediction = propdetect.left;
        } else if (perecentMiddle > perecentLeft && perecentMiddle > percentRight && perecentMiddle > min ) {
            prediction = propdetect.middle;
        } else if (percentRight > perecentLeft && percentRight > perecentMiddle && percentRight > min ) {
            prediction = propdetect.right;
        }else{
            prediction = propdetect.error;
            telemetry.addLine("nothing found :(");
        }
        return null;
    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // NA
    }

    public propdetect getPrediction(){
        return prediction;
    }

    public enum propdetect{
        left(1),
        middle(2),
        right(3),
        error(0);
        public final int pos_num;
        propdetect(int posNum) {
            pos_num = posNum;
        }
    }
}

