package org.firstinspire.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspire.ftc.teamcode.subsystems.GoBildaPinpointDriver;

public class HardwareRobot {
    public final MotorEx leftFront;
    public final MotorEx rightFront;
    public final MotorEx leftBack;
    public final MotorEx rightBack;



    public final Servo claw;
    public final MotorEx elbow;
    public final MotorEx elbowtwo;
    public final Servo clawrotation;
    public final WebcamName webcamName;

    public HardwareRobot(HardwareMap hardwareMap) {

        ////////////
        // WHEELS //
        ////////////
        leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(false);
        rightFront.setInverted(false);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //check rpm for elbow
        elbow = new MotorEx(hardwareMap, "Elbow", Motor.GoBILDA.RPM_30);
        elbow.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setRunMode(Motor.RunMode.RawPower);
        elbow.setInverted(true);
        elbow.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elbowtwo = new MotorEx(hardwareMap, "elbowtwo", Motor.GoBILDA.RPM_30);
        elbowtwo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowtwo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowtwo.setRunMode(Motor.RunMode.RawPower);
        elbowtwo.setInverted(true);
        elbowtwo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        /////////////
        // SERVOS  //
        /////////////
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        clawrotation = hardwareMap.get(ServoImplEx.class, "Rotation");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
    public void changeInversions() {
        leftFront.setInverted(true);
        rightBack.setInverted(true);
    }
}