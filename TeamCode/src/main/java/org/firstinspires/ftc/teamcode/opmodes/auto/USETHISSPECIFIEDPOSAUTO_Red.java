package org.firstinspires.ftc.teamcode.opmodes.auto;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CvSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;
public class USETHISSPECIFIEDPOSAUTO_Red extends LinearOpMode {
    public RobotSystem robot;
    //vision portal and processor initialization
    private CvSubsystem visionPipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
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
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        this.robot = new RobotSystem(hardwareMap, this);
        CvSubsystem.setIsread(true);
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionPipeline =  new CvSubsystem();
        camera.setPipeline(visionPipeline);
        this.robot = new RobotSystem(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            aprilTagDetect(tagProcessor);
            relativeDrive(tagProcessor, 6, 200, 700,100,100);
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
                    telemetry.addData("X", tag.ftcPose.x);
                    telemetry.addData("Y", tag.ftcPose.y);
                    telemetry.addData("Z", tag.ftcPose.z);
                }
            } else {
                telemetry.addLine("No Tag Detected.");
            }
            telemetry.update();
        }
    }

    /**info on field coordinate system:
     * Origin
     * The 0,0,0 origin of the FIRST Tech Challenge coordinate system is the point in the center of the field, equidistant from all 4 perimeter walls (where the four center tiles meet). The origin point rests on the top surface of the floor mat.
     * X Axis
     * Looking at the origin from the Red Wall, the X axis extends through the origin point and runs to the right and left, parallel with the Red Wall. The X axis values increase to the right.
     * Y Axis
     * Looking at the origin from the Red Wall, the Y axis extends through the origin point and runs out and in, perpendicular to the Red Wall. Increasing Y values run out (away) from the Red Wall.
     * Z Axis
     * Looking at the origin from the Red Wall, the Z axis extends through the origin point and runs up and down in a vertical line. Increasing Z values extend upwards.
     */
    public void relativeDrive (AprilTagProcessor processor, int SpecTag, double tagFieldX, double tagFieldY, double targetFieldX, double targetFieldY) {
        //btw max coords are 144,144 (field is 12ft by 12ft)
        List<AprilTagDetection> detection = processor.getDetections();
        for (AprilTagDetection tag : detection) {
            if (tag.id == SpecTag) {
                double robotFieldX = tag.robotPose.getPosition().x;
                double robotFieldY = tag.robotPose.getPosition().y;
                double errorX = targetFieldX - robotFieldX;
                double errorY = targetFieldY - robotFieldY;
                double kP = 0.01;
                double kD = 0.001;
                double previousErrorX = 0;
                double previousErrorY = 0;
                ElapsedTime timer = new ElapsedTime();
                while (opModeIsActive() && Math.hypot(errorX, errorY) > 0.5) {  // 0.5 is an acceptable error threshold
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

                    errorX = targetFieldX - robotFieldX;
                    errorY = targetFieldY - robotFieldY;
                }
            }
        }
    }
}
