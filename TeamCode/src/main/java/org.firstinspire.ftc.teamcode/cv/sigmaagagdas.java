package org.firstinspire.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
//213, 320
public class sigmaagagdas extends OpenCvPipeline {
    Rect ch1 = new Rect(50, 100, 20,20);
    Rect ch2 = new Rect(175, 100, 20,20);
    Scalar red = new Scalar(255,0,0);
    Rect ch3 = new Rect(250, 100, 20,20);
    public Scalar upper = new Scalar(150,255,255);
    public Scalar lower = new Scalar(125,50,50);
    @Override
    public Mat processFrame(Mat img) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_RGB2HSV);
        boolean first = checkRegion(hsv, ch1, lower, upper, 0.8);
        boolean second = checkRegion(hsv, ch2, lower, upper, 0.8);
        boolean third = checkRegion(hsv, ch3, lower, upper, 0.8);
        Imgproc.putText(
                img,                 // frame to draw on
                "Seq: " + getMotif(first, second, third),    // text
                new Point(50, 50),     // position
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,                   // scale
                new Scalar(255, 0, 0), // color (BGR)
                2                      // thickness
        );
        return img;
    }
    public String getMotif(boolean one, boolean two, boolean three) {
        StringBuilder sb = new StringBuilder();
        if (one) {
            sb.append("P");
        } else {
            sb.append("G");
        }
        if (two) {
            sb.append("P");
        } else {
            sb.append("G");
        }
        if (three) {
            sb.append("P");
        } else {
            sb.append("G");
        }
        return sb.toString();
    }
    public boolean checkRegion(Mat src, Rect roi, Scalar lower, Scalar upper, double minFraction) {
        Mat submat = src.submat(roi);
        Mat mask = new Mat();
        Core.inRange(submat, lower, upper, mask);
        int totalpixels = roi.width * roi.height;
        int nonzero = Core.countNonZero(mask);
        double fractionInRange = (double) nonzero / totalpixels;
        return fractionInRange >= minFraction;
    }
}
