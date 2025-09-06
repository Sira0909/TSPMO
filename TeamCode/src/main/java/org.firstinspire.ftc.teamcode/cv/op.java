package org.firstinspire.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class op extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Black magic (check official docs if you want to know what this actually does)
        // Used for displaying the pipeline output on the phone.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // 320x240 is the resolution. You can change the rotation if your phone is mounted weirdly.
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Make the camera process frames using the pipeline you just wrote
        camera.setPipeline(new pipeline());

        while(opModeIsActive()) {

        }

        // Stop everything
        camera.closeCameraDevice();
    }
}
