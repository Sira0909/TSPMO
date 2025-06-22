package org.firstinspire.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
//possibly add imu and see how that works
//6/23: work on pd and boolean logic
@Autonomous (name = "Test")
public class TSPMOAuto extends LinearOpMode {
    private RobotSystem robot;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;
    public AprilTagDetection lastDetectedTag;
    public boolean control = false;
    public boolean driveCompleted = false;
    public boolean updateControl;
    public ElapsedTime runtime;
    public double speed = 0.45;
    public int lastTime = 0;
    public int lastError = 0;
    public void reset() {
        lastError = 0;
        lastTime = 0;
        runtime.reset();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        this.runtime = new ElapsedTime();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.robot = new RobotSystem(hardwareMap, this);
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();
        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .setCamera(robot.hardwareRobot.webcamName)
                .setLiveViewContainerId(cameraMonitorViewId)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        while (opModeIsActive()) {
            /// Coordinate System Explanation:
            /// Any detected tag immediately becomes the origin.
            /// The graph consists of the 3rd and 4th quadrants.
            /// Any negative returned by the x value of a tag detected by a camera will place the robot in the 3rd quadrant.
            /// Returning positive x values for tag detection places the robot in the 4th quadrant.
            /// Y can only be negative.
            detectTags();
            if (Proximity(lastDetectedTag, 6)) {
                speed += -0.2;
            }
            updateControl = control;
        }
    }
    public void detectTags () {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) {
            lastDetectedTag = null;
            return;
        }
        for (AprilTagDetection tag : detections) {
            telemetry.addLine("AprilTag Detected.");
            telemetry.addData("X:", tag.ftcPose.x);
            telemetry.addData("Y:", tag.ftcPose.y);
            telemetry.addData("Z:", tag.ftcPose.z);
            telemetry.addData("Bearing:", tag.ftcPose.bearing);
            telemetry.addData("Yaw:", tag.ftcPose.yaw);
            telemetry.addData("Range:", tag.ftcPose.range);
            telemetry.addData("ID:", tag.id);
            lastDetectedTag = tag;
            break;
        }
    }
    public boolean Proximity (AprilTagDetection targetTag, int radius) {
        if (lastDetectedTag == null) {
            return false;
        }
        return targetTag.ftcPose.range < radius;
    }
    public void PD (AprilTagDetection target) {
        if (!driveCompleted) {
            control = true;
        }
    }
}
