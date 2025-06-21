package org.firstinspire.ftc.teamcode.auto;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class TSPMOAutop extends LinearOpMode {
    private RobotSystem robot;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;
    @Override
    public void runOpMode() throws InterruptedException {
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
            /// The graph consists of the 1st and 2nd quadrants.
            /// Any negative returned by the x value of a tag detected by a camera will place the robot in the 1st quadrant.
            /// Returning positive x values for tag detection places the robot in the 2nd quadrant.
            /// Y can only be positive.

        }
    }
}
