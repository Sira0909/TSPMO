package org.firstinspire.ftc.teamcode.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspire.ftc.teamcode.RobotConstants;
//down + up rotation are different from picking up pos and depositing pos.
// x is toggle claw, triangle is middle rotation, square is down rotation, circle is up rotation. left bumper is macro 1 and right bumper is macro 2
//TODO: add macro w error for going down and opening claw, macro for picking up and raising, macro for raising even more. claw should be opened separately.
@TeleOp (name = "StemtasticTeleOp")
public class StemtasticTeleOp extends LinearOpMode {
    public RobotSystem robot;
    public double speed;
    public double rotationPos;
    public double clawPos;
    public boolean toggleClaw = false;
    public boolean claw = true;
    @Override
    public void runOpMode () throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.inDep.setElbowPos(0);
        speed = 0.6;
        robot.inDep.setClawPosition(0);
        robot.inDep.setRotationPos(RobotConstants.CLAWROTATIONMIDDLE);
        rotationPos = 0.6;
        clawPos = 0;
        waitForStart();
        while(opModeIsActive()) {
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPos(rotationPos);
            double elbowPower = gamepad1.right_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double forward = -gamepad1.left_stick_y;
            robot.drive.driveRobotCentric(strafe  * speed, forward  * speed, turn  * speed);
            robot.inDep.setElbowPos(elbowPower);
            //for this we might follow a decrease along a graph but this is just a placeholder
            if (Math.abs(elbowPower) > 0.2 ) {
                turn = 0;
            }
            else if (Math.abs(turn) > 0.2) {
                elbowPower = 0; //or whatever regular elbow is
            }
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.addData("Elbow Position: ", elbowPower);
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.update();
            // the following work as toggle - we wouldnt even need to use them unless were showing the movement to ppl
            // this is because otherwise we would just use the macros
            if (gamepad1.square) {
                rotationPos = RobotConstants.CLAWROTATIONUP;
            }
            else {
                rotationPos = RobotConstants.CLAWROTATIONMIDDLE;
            }
            if (gamepad1.circle) {
                rotationPos = RobotConstants.CLAWROTATIONDOWN;
            }
            else {
                rotationPos = RobotConstants.CLAWROTATIONMIDDLE;
            }
            //now for claw pressing once - we need to use it in action.
            double elbowpower = gamepad1.right_stick_y;
            robot.inDep.setElbowPos(elbowpower);
            if (gamepad1.dpad_right && !toggleClaw) {
                claw = !claw;
                toggleClaw = true;
            }
            if (!gamepad1.dpad_right) {
                toggleClaw = false;
                claw = true;
            }
            if (claw) {
                clawPos = RobotConstants.CLOSECLAW;
            }
            else {
                clawPos = RobotConstants.OPENCLAW;
            }
        }
    }
}
