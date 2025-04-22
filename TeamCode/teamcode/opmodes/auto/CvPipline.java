package teamcode.opmodes.auto;

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

public class CvPipline extends OpenCvPipeline implements VisionProcessor {

    private Rect leftSide;
    private Rect middle;
    private Rect rightSide;

    private final Mat HSV = new Mat();
    private final Mat low = new Mat();
    private final Mat high = new Mat();
    private final Mat range = new Mat();

    private Scalar upperUpper;
    private Scalar upperLower;
    private Scalar lowerUpper;
    private Scalar lowerLower;

    private double percentLeft;
    private double percentRight;
    private double percentMiddle;

    private static boolean isRed = false;
    private final int min = 10;

    private PropDetect prediction = PropDetect.ERROR;

    public static void setIsRed(boolean x) {
        isRed = x;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (isRed) {
            upperUpper = new Scalar(180, 50, 50);
            upperLower = new Scalar(170, 50, 50);
            lowerUpper = new Scalar(20, 50, 50);
            lowerLower = new Scalar(0, 50, 50);
        } else {
            upperUpper = new Scalar(130, 255, 255);
            upperLower = new Scalar(110, 50, 50);
            lowerUpper = new Scalar(130, 255, 255);
            lowerLower = new Scalar(110, 50, 50);
        }

        leftSide = new Rect(new Point(0, 0), new Point(0.33 * width, height));
        middle = new Rect(new Point(0.33 * width, 0), new Point(0.66 * width, height));
        rightSide = new Rect(new Point(0.66 * width, 0), new Point(width, height));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }


    @Override
    public Mat processFrame(Mat frame) {
        // Convert to HSV color space
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_BGR2HSV);
        Core.inRange(HSV, upperLower, upperUpper, high);
        Core.inRange(HSV, lowerLower, lowerUpper, low);
        Core.bitwise_or(low, high, range);

        // Calculate percentages for each region
        percentLeft = (Core.sumElems(range.submat(leftSide)).val[0] / leftSide.area()) * 100;
        percentRight = (Core.sumElems(range.submat(rightSide)).val[0] / rightSide.area()) * 100;
        percentMiddle = (Core.sumElems(range.submat(middle)).val[0] / middle.area()) * 100;

        // Determine the prediction based on max detection
        if (percentLeft > percentMiddle && percentLeft > percentRight && percentLeft > min) {
            prediction = PropDetect.LEFT;
        } else if (percentMiddle > percentLeft && percentMiddle > percentRight && percentMiddle > min) {
            prediction = PropDetect.MIDDLE;
        } else if (percentRight > percentLeft && percentRight > percentMiddle && percentRight > min) {
            prediction = PropDetect.RIGHT;
        } else {
            prediction = PropDetect.ERROR;
            if (telemetry != null) {
                telemetry.addLine("Nothing found :(");
            }
        }

        // Draw rectangles based on prediction
        if (prediction == PropDetect.LEFT) {
            Imgproc.rectangle(frame, leftSide, lowerUpper);
        } else if (prediction == PropDetect.MIDDLE) {
            Imgproc.rectangle(frame, middle, lowerUpper);
        } else if (prediction == PropDetect.RIGHT) {
            Imgproc.rectangle(frame, rightSide, lowerUpper);
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not implemented
    }

    public PropDetect getPrediction() {
        return prediction;
    }

    public enum PropDetect {
        LEFT(1),
        MIDDLE(2),
        RIGHT(3),
        ERROR(0);

        public final int posNum;

        PropDetect(int posNum) {
            this.posNum = posNum;
        }
    }
}
