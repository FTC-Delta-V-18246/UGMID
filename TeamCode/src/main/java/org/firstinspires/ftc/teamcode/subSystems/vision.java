package org.firstinspires.ftc.teamcode.subSystems;


import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.UGBuildSeason.UGContourRingPipe;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static java.lang.Thread.sleep;

@Config
public class vision {
    public static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune
    public static int HORIZONX = 30;
    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    UGContourRingPipe pipeline;
    public UGAngleHighGoalPipeline goalline;
    private OpenCvCamera camera;
    private LinearOpMode opMode;
    private boolean stack = true;
    public static double lf = 0.0, ls = 147.0, lt =0.0;
    public static double hf = 255.0, hs = 189.0, ht = 120.0;
    public static Scalar lowBound = new Scalar(lf, ls, lt);
    public static Scalar highBound = new Scalar(hf,hs, ht);
    public static int minWidth = 33;
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

        camera.setPipeline(pipeline = new UGContourRingPipe(opMode.telemetry, DEBUG));
        //UGContourRingPipe.Config.setLowerOrange(new Scalar(lf, ls, lt));
        //UGContourRingPipe.Config.setUpperOrange(new Scalar(hf,hs, ht));
        //UGContourRingPipeline.Config.setLowerOrange(new Scalar(0.0, 0, 0.0));
        //UGContourRingPipeline.Config.setUpperOrange(new Scalar(255.0,255.0, 255.0));

        UGContourRingPipe.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        UGContourRingPipe.Config.setHORIZON(HORIZON);
        UGContourRingPipe.Config.setHORIZONX(HORIZONX);
       camera.openCameraDevice();
       camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);



        // states camera code
        //camera.openCameraDeviceAsync(() ->{
          //  camera.openCameraDevice();
           // camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
        //});



        /*testing code
        camera.openCameraDevice();
        camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
        */


    }
    public int height(){
        if(stack) {
            switch (pipeline.getHeight()) {
                case ONE:
                    return 1;
                case ZERO:
                    return 0;
                case FOUR:
                    return 4;
            }
        }
        return 69;
    }
    public static void updateColor(){
       //UGContourRingPipe.Config.setLowerOrange(lowBound);
        //UGContourRingPipe.Config.setUpperOrange(highBound);
        UGContourRingPipe.Config.setMIN_WIDTH(minWidth);


    }
    public void highGoal(){
        stack = false;
        camera.setPipeline(goalline = new UGAngleHighGoalPipeline(opMode));
    }
}
