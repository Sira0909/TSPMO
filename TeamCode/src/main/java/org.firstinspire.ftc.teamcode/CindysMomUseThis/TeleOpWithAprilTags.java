package org.firstinspire.ftc.teamcode.CindysMomUseThis;
//Import statements.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //Allows us to create main opmode loop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; //Notation so that this opMode appears on the driver hub.

import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspire.ftc.teamcode.RobotSystem;
//Deal with apriltag processing.
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
//Note: there was a method in development called "driveToTag" that used a PD controller to drive toward a specific tag; however, it was removed.
//Feel free to try and design a method of your own dealing with this movement!
//Additionally, if you wish to view the camera's live feed, you will have to download scr-cpy for android and run it once you are connected to the control hub.
/// Controls:
/// Square: rotation, dpad left: claw, right x: turn, right y: elbow, left x: strafe, left y: forward.
@TeleOp (name = "TeleOpWithAprilTags")
public class TeleOpWithAprilTags extends LinearOpMode {
    public RobotSystem robot;
    public double rotationPos;
    public double clawPos;
    public int encoderPos;
    public double elbowPower;
    private boolean clawOpen = true;
    private boolean dpadPressedLastLoop = false;
    private boolean rotationDown = true;
    private boolean squarePressedLastLoop = false;
    public double speed = 0.45;
    public AprilTagDetection lastTagDetected;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        //Allows for the live view of the camera.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        //Tag Processor = acts on info coming from visionPortal and performs a series of processes on the images, and then returns the processed frames back.
        this.tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(robot.hardwareRobot.webcamName)
                .setAutoStopLiveView(false)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .setLiveViewContainerId(cameraMonitorViewId)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        waitForStart();
        while (opModeIsActive()) {
            encoderPos = robot.inDep.getEncoder(encoderPos);
            //Has to be reduced - will break elbow if at full power.
            elbowPower = gamepad1.right_stick_y * 0.5;
            //Issues with the led me to invert some of the joysticks.
            double strafe = -gamepad1.left_stick_y;
            double forward = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            //The same problems occurred with the wheels.
            if (Math.abs(turn) > 0) {
                robot.hardwareRobot.changeInversions();
            }
            //Detects proximity to closest tag and decreases speed if in given range of the tag.
            //(this method was originally to be used with the drivetotag method)
            boolean proximity = xInchRadius(lastTagDetected, 20);
            if (proximity) {
                speed += -0.15;
            }
            detectTags();
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            /// Boolean Logic Explanation:
            /// The claw and rotation do NOT work off of toggle.
            /// Initially, both are false. Since the clawPos was set to closed earlier, it stays closed.
            /// The next loop, if the dpad is pressed, the condition holds. clawOpen's value flips, opening the claw.
            /// If it is let go and pressed again, the same thing happens, closing the claw.
            /// So, Separate presses are needed to alternate the claw's position.
            boolean isDpadPressed = gamepad1.dpad_right;
            if (isDpadPressed && !dpadPressedLastLoop) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    clawPos = RobotConstants.OPENCLAW;
                } else {
                    clawPos = RobotConstants.CLOSECLAW;
                }
            }
            //Same logic as above.
            boolean isSquarePressed = gamepad1.square;
            if (isSquarePressed && !squarePressedLastLoop) {
                rotationDown = !rotationDown;
                if (rotationDown) {
                    rotationPos = RobotConstants.CLAWROTATIONPIKINGUP;
                } else {
                    rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
                }
            }
            //Setting all updated positions
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            //Sticks are -1 to 1, by the way.
            robot.inDep.setElbowPosition(elbowPower);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.addData("Encoder Position: ", encoderPos);
            //Makes sure no nullPointerException is thrown.
            //Note that if any tags from past seasons are shown, the same error will be thrown.
            if (lastTagDetected != null) {
                telemetry.addData("Proximity to closest tag:", xInchRadius(lastTagDetected, 20));
            }
            telemetry.update();
            //updating variables
            dpadPressedLastLoop = isDpadPressed;
            squarePressedLastLoop = isSquarePressed;
        }
    }
    /// This method works. It iterates through each detection (assuming only one happens at a time),
    /// displaying different values. X: horizontal offset to the center of the tag. Y: forwards/backwards. Z: vertical offset.
    /// Range: direct distance from center of camera to center of tag. Bearing: angle at which the camera is turned toward the tag.
    /// Yaw: 0 is perpendicular to the tag.
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                telemetry.addLine("AprilTag Detected.");
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                telemetry.addData("Y (Forward/Back Offset)", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Range: ", tag.ftcPose.range);
                lastTagDetected = tag;
                break;
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
    public boolean xInchRadius(AprilTagDetection taggg, double radius) {
        if (lastTagDetected == null) {
            return false;
        }
        else {
            return taggg.ftcPose.range <= radius;
        }
    }
}
