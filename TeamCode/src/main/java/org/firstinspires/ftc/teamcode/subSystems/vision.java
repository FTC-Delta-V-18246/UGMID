package org.firstinspires.ftc.teamcode.subSystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
public class vision {
    public static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static int HORIZON = 0; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;
    private LinearOpMode opMode;
    public vision(LinearOpMode opMode){
        this.opMode = opMode;
        int cameraMonitorViewId = this
                .opMode.hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        opMode.hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(opMode.hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(opMode.telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN));
    }
    public int height(){
        switch (pipeline.getHeight()){
            case ONE:
                return 1;
            case ZERO:
                return 0;
            case FOUR:
                return 4;
        }
        return 69;
    }
}
