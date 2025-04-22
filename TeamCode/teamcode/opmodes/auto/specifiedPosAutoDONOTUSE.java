package teamcode.opmodes.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//NOTE: THIS IS A DUMB FILE THAT OVERCOMPLICATES EVERYTHING. IM SAVING IT BC IM PROUD OF IT.
public class specifiedPosAutoDONOTUSE extends LinearOpMode {
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
            .setStreamFormat(VisionPortal.StreamFormat.YUY2)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            aprilTagDetect(tagProcessor);
            relativeDrive(tagProcessor, 6, 200, 700,100,100);

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
                    getposrelativetoapriltag(tagProcessor, tag.id);
                }
            } else {
                telemetry.addLine("No Tag Detected.");
            }
            telemetry.update();
        }
    }
    public void alignRobotToTag(AprilTagProcessor tagProcessor, int tagID) {

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
        Double[] robotRelative = getposrelativetoapriltag(tagProcessor, SpecTag);
        if (robotRelative == null) {
            telemetry.addLine("No AprilTag detected!");
            telemetry.update();
            //f
        }
        else {
            //btw max coords are 144,144 (field is 12ft by 12ft)
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
    //ARI's METHOD: not used
    //coords are relative to bottom left corner
    //public double[] getpositiononfeild(AprilTagProcessor tagProcessor){
    //ArrayList<Double[]> detectedpositions = new ArrayList<>();
    //List<AprilTagDetection> detections = tagProcessor.getDetections();
    //for (AprilTagDetection tag : detections) {
    //double complementarybearing = 90 - tag.ftcPose.bearing;
    //double tagangletorobot = tag.ftcPose.yaw+complementarybearing;
    //double radangle = -tagangletorobot*Math.PI/180;
    //double robotrelativex=Math.cos(radangle)*tag.ftcPose.range;
    //double robotrelativey=Math.sin(radangle)*tag.ftcPose.range;
    //double[] apriltagpositions = getapriltagpositions(tag.id);

    //trig magic
    //double robotfieldrelativex = robotrelativex*Math.cos(radangle)+robotrelativey*Math.sin(radangle);
    //double robotfieldrelativey = robotrelativey*Math.cos(radangle)-robotrelativex*Math.sin(radangle);
    //dont ask why that works i dont know. it just does. something about trig i think.
    //double robotfieldx = tag.metadata.fieldPosition.get(0)+robotfieldrelativex;
    //double robotfieldy = apriltagpositions[1]+robotfieldrelativey;

    //detectedpositions.add(new Double[]{tag.robotPose.getPosition().x,robotfieldy});
    //}
    //double avgx = 0;
    //double avgy = 0;
    //for(int i =0;i<detectedpositions.size();i++){
    //avgx+= detectedpositions.get(i)[0]/detectedpositions.size();
    //avgy+= detectedpositions.get(i)[1]/detectedpositions.size();
    //}
    //return new double[] {avgx,avgy};
    //}
}
