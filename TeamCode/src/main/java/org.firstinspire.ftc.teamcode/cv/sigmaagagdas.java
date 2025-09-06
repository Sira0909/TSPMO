package org.firstinspire.ftc.teamcode.cv;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
//238 * 240
    public class sigmaagagdas extends OpenCvPipeline {
        Rect r = new Rect(50, 120, 20,20);
        Rect rr = new Rect(100, 120, 20,20);
        Scalar red = new Scalar(255,0,0);
        Rect rrr = new Rect(130, 120, 20,20);

    @Override
        public Mat processFrame(Mat img) {
            Imgproc.rectangle(img, r, red);
            Imgproc.rectangle(img, rr, red);
            Imgproc.rectangle(img, rrr, red);
            return img;
        }
    }
