package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//if a new apriltag class needs to be create, use this.
public class aprilTagsTemplate extends LinearOpMode {
    public void aprilTagDetect(AprilTagProcessor tagProcessor) {
        if (true) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                telemetry.addLine("AprilTags Detected:");
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.range);
                    telemetry.addData("Distance", tag.ftcPose.range);
                    telemetry.addData("Yaw", tag.ftcPose.yaw);
                    //now im forced to do auto inside of this... fix this later bc it would do pd controller for every tag
                    double error = tag.ftcPose.range;

                }
            } else {
                telemetry.addLine("No Tag Detected.");
            }
            telemetry.update();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder() //creates object of processor class for detection\
                //calling set up methods - drawing and mapping out possible predicted tags
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //build intializes all of these
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder() //actual cv program built in
                //adding processor inside of camera display to detect tags
                .addProcessor(tagProcessor)
                //adding cam to do the actual detection
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //setting cam positions
                .setCameraResolution(new Size(640, 480)) // place holder values ask for real size
                .build();
        while(opModeIsActive()) {
            aprilTagDetect(tagProcessor);

        }
    }
}
