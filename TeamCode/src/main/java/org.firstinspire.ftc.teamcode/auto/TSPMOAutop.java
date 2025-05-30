package org.firstinspire.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TSPMOAutop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
