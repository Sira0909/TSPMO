package org.firstinspire.ftc.teamcode.cv;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

    public class iwajef0ipawjef extends OpenCvPipeline {
        Mat mask = new Mat();
        Mat ret = new Mat();
        Scalar red = new Scalar(255, 0, 0);
        @Override
        public Mat processFrame(Mat img) {
            Rect r = new Rect(150, 500, 100,100);
            Rect rr = new Rect(500, 500, 100, 100);
            Rect rrr = new Rect(750, 500, 100, 100);
            Imgproc.rectangle(img, r, red);
            Imgproc.rectangle(img, rr, red);
            Imgproc.rectangle(img, rrr, red);
            return img;
        }
    }
