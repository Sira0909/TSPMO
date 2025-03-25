package org.firstinspires.ftc.teamcode.opmodes.auto;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

//this will allow movement from position relative to apriltag to another position relative to that apriltag
public class apriltagPDrelativetotags extends LinearOpMode {
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
                    double tagangletorobot = tag.ftcPose.yaw+complementarybearing;
                    double robotrelativex=Math.sin(-tagangletorobot*Math.PI/180)*tag.ftcPose.range;
                    double robotrelativey=Math.cos(-tagangletorobot*Math.PI/180)*tag.ftcPose.range;
                    telemetry.addData("robotrelativex",robotrelativex);
                    telemetry.addData("robotrelativey",robotrelativey);
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
                    double tagangletorobot = tag.ftcPose.yaw+complementarybearing;
                    double robotrelativex=Math.sin(-tagangletorobot*Math.PI/180)*tag.ftcPose.range;
                    double robotrelativey=Math.cos(-tagangletorobot*Math.PI/180)*tag.ftcPose.range;
                    return new Double[]{robotrelativex, robotrelativey};
                }
            }
        }
        return null;
    }

    public void changepositionrelativetotag(AprilTagProcessor tagProcessor, int tagid, int targetx, int targetrange,double guessstartx, double guessstarty){
        double GOALERROR = 0.01;
        double KP = 0.01;
        double KD = 0.001;
        Double[] currentposrelative = getposrelativetoapriltag(tagProcessor,tagid);
        if(currentposrelative==null){currentposrelative = new Double[]{guessstartx,guessstarty};}
        Double currenterror = Math.hypot(Math.abs(currentposrelative[0]-targetx),Math.abs(currentposrelative[1]-targetrange));
        Double errorlist[] = {currenterror,currenterror,currenterror,currenterror,currenterror,currenterror,currenterror,currenterror,currenterror,currenterror};
        ElapsedTime runtime = new ElapsedTime(0);
        double timelist[] = {runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds(),runtime.seconds()};
        double lastD;
        while (currenterror>GOALERROR){


            //NOTE: I HAD AN IDEA FOR HOW TO DO THIS, FEEL FREE TO CHANGE AND OR REMOVE. THE METHOD
            //I WAS WRITING IS UNFINISHED, DONT TRUST IT



            currentposrelative = getposrelativetoapriltag(tagProcessor,tagid);
            if(currentposrelative==null){currenterror=null;}else{
            currenterror = Math.hypot(Math.abs(currentposrelative[0]-targetx),Math.abs(currentposrelative[1]-targetrange));}
            ArrayList<Integer> validtags= new ArrayList<Integer>();
            for (int i = 0;i<errorlist.length-1;i++){
                errorlist[i]=errorlist[i+1];
                if(errorlist[i]!=null){validtags.add(i);}
                timelist[i]=timelist[i+1];
            }
            errorlist[errorlist.length-1]=currenterror;
            if(currentposrelative!=null){validtags.add(errorlist.length);}

            timelist[timelist.length]=runtime.seconds();

            if(validtags.size()>=5){

            }



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
