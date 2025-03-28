package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.List;

//adding documentation and fixing apriltag stuff - viir 3/20
public class BasicTeleopForTSPMO extends LinearOpMode {

    //creates robot as object of compiled robotsystem class w all subsystems
    //  ======    |\\    ||  ======  =========
    //    ||      ||\\   ||    ||       ||
    //    ||      || \\  ||    ||       ||
    //    ||      ||  \\ ||    ||       ||
    //    ||      ||   \\||    ||       ||
    //  ======    ||    \\|  ======     ||
    public RobotSystem robot;

    boolean toggleClaw = false;
    boolean claw = true;
    private final RobotConstants ROBOTCONSTANTS = new RobotConstants();


    private double PEX = 0;
    private double PEY = 0;
    private double PEYAW = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        //define robot object
        this.robot = new RobotSystem(hardwareMap, this);
        //assuming elbow has infinite positions
        //ari also wanted this
        //setting pos for claw and elbow
        robot.inDep.closeClaw();
        robot.inDep.setElbowPos(0);//test servo positions once accessible
        // test servo pos as well


        //apriltag intialization
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


        //  /=====    ========        /\       |====\    ========
        // |             ||          //\\      |     |      ||
        //  \====\       ||         //  \\     |====/       ||
        //        |      ||        //====\\    |  \\        ||
        //  =====/       ||       //      \\   |   \\       ||

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            liftCommands();
            letterbuttons();
        }
    }


    //  ||       ========   |======  ========
    //  ||          ||      ||          ||
    //  ||          ||      |=====      ||
    //  ||          ||      ||          ||
    //  ======   ========   ||          ||
    public void liftCommands() {
        double triggerPower = (gamepad1.left_trigger - gamepad1.right_trigger);
        robot.inDep.setLiftPos(triggerPower);
        //make lift up and lift down macros if needed - viir has them already
    }

    public void driveCommands() {
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }

    public void letterbuttons() {
        double elbowpower = gamepad1.right_stick_y;
        robot.inDep.setElbowPos(elbowpower);
        if (gamepad1.circle && !toggleClaw) {
            toggleClaw = true;
            claw = !claw;
        }
        if (!gamepad1.circle) {
            toggleClaw = false;
        }
        if (claw) {
            robot.inDep.closeClaw();
        } else {
            robot.inDep.openClaw();
        }
        if (gamepad1.cross) {
            //lift up macro
        }
        if (gamepad1.triangle) {
            //lift down macro
        }

    }

                }
            }
        }
        if (target != null) {
            PDcontroller(target);
        }
    }
}
    public void PDcontroller(AprilTagDetection target){
        double kP = 0.1; double kD = 0.01; // Tune these obv

        double errorX = target.ftcPose.x;
        double errorY = target.ftcPose.y - 1; // subtract 1 bc we dont wanna crash into the tag
        double errorYaw = target.ftcPose.yaw;

        double derivativeX = errorX -PEX ;
        double derivativeY = errorY - PEY;
        double derivativeYaw = errorYaw - PEYAW;

        double strafePower = kP * errorX + kD * derivativeX;
        double forwardPower = kP * errorY + kD * derivativeY;
        double turnPower = kP * errorYaw + kD * derivativeYaw;

        //CONVERTING STRafe FORWARD AND TUIRN POWER INTO -1 to 1 range
        strafePower = Math.max(-1, Math.min(1, strafePower));
        forwardPower = Math.max(-1, Math.min(1, forwardPower));
        turnPower = Math.max(-1, Math.min(1, turnPower));

        robot.drive.driveRobotCentric(strafePower, forwardPower, turnPower);

        PEX = errorX; PEY = errorY; PEYAW = errorYaw;
    }
}




