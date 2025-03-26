package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class specifedPosAuto extends LinearOpMode {
    public RobotSystem robot;
    //vision portal and processor initialization
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .build();
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            aprilTagDetect(tagProcessor);
            if (gamepad1.cross) {
                relativeDrive(tagProcessor, 6, 200, 700,100,100);
            }
        }
    }

    public void aprilTagDetect(AprilTagProcessor tagProcessor) {
        if (true) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                telemetry.addLine("AprilTags Detected:");
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Distance", tag.ftcPose.range);
                    double error = tag.ftcPose.range;
                    double complementarybearing = 90 - tag.ftcPose.bearing;
                    double tagangletorobot = tag.ftcPose.yaw + complementarybearing;
                    double robotrelativex = Math.sin(-tagangletorobot * Math.PI / 180) * tag.ftcPose.range;
                    double robotrelativey = Math.cos(-tagangletorobot * Math.PI / 180) * tag.ftcPose.range;
                    telemetry.addData("robotrelativex", robotrelativex);
                    telemetry.addData("robotrelativey", robotrelativey);
                }
            } else {
                telemetry.addLine("No Tag Detected.");
            }
            telemetry.update();
        }
    }

    public Double[] getposrelativetoapriltag(AprilTagProcessor tagProcessor, int tagid) {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == tagid) {
                    double complementarybearing = 90 - tag.ftcPose.bearing;
                    double tagangletorobot = tag.ftcPose.yaw + complementarybearing;
                    double robotrelativex = Math.sin(-tagangletorobot * Math.PI / 180) * tag.ftcPose.range;
                    double robotrelativey = Math.cos(-tagangletorobot * Math.PI / 180) * tag.ftcPose.range;
                    return new Double[]{robotrelativex, robotrelativey};
                }
            }
        }
        return null;
    }

    public void relativeDrive (AprilTagProcessor processor, int SpecTag, double tagFieldX, double tagFieldY, double targetFieldX, double targetFieldY) {
        Double[] robotRelative = getposrelativetoapriltag(tagProcessor, SpecTag);
        if (robotRelative == null) {
            telemetry.addLine("No AprilTag detected!");
            telemetry.update();
        }
        else {
            double robotFieldX = tagFieldX + robotRelative[0];
            double robotFieldY = tagFieldY + robotRelative[1];
            double errorX = targetFieldX - robotFieldX;
            double errorY = targetFieldY - robotFieldY;
            double kP = 0.01;
            double kD = 0.001;
            double previousErrorX = 0;
            double previousErrorY = 0;
            ElapsedTime timer = new ElapsedTime();
            while (Math.hypot(errorX, errorY) > 0.5) {  // 0.5 is an acceptable error threshold
                double deltaTime = timer.seconds();
                timer.reset();

                // PD Control
                double derivativeX = (errorX - previousErrorX) / deltaTime;
                double derivativeY = (errorY - previousErrorY) / deltaTime;

                double powerX = kP * errorX + kD * derivativeX;
                double powerY = kP * errorY + kD * derivativeY;

                robot.drive.driveRobotCentric(powerX, powerY, 0);  // Replace with your robot’s drive method

                previousErrorX = errorX;
                previousErrorY = errorY;

                robotRelative = getposrelativetoapriltag(tagProcessor, SpecTag);

                robotFieldX = tagFieldX + robotRelative[0];
                robotFieldY = tagFieldY + robotRelative[1];

                errorX = targetFieldX - robotFieldX;
                errorY = targetFieldY - robotFieldY;
            }
        }
    }
}
