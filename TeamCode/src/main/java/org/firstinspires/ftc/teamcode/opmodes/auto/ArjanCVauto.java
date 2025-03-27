package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="good auto (sample)")
public class ArjanCVauto extends LinearOpMode{
    public RobotSystem robot;
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);

        waitForStart();

        while (opModeIsActive()) {
            
        }
    }
}
