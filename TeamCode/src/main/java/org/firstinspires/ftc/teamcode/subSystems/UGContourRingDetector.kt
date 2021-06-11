
package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.UGBuildSeason.UGContourRingPipe
import org.firstinspires.ftc.teamcode.UGBuildSeason.UGContourRingPipe.Config.lowerOrange
import org.firstinspires.ftc.teamcode.UGBuildSeason.UGContourRingPipe.Config.upperOrange
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

/**
 * UGContourRingPipeline Detector Class
 *
 * @author Jaran Chao
 *
 * @constructor Constructs the Vision Detector
 *
 * @param hardwareMap variable being passed into the detector to access hardwareMap, no default value
 * @param cameraDirection direction of the camera if using phone camera (FRONT, BACK), default value
 * to BACK
 * @param telemetry If wanted, provide a telemetry object to the constructor to get stacktrace if an
 * error occurs (mainly for debug purposes), default value of null
 * @param debug If true, all intermediate calculation results (except showing mat operations) will
 * be printed to telemetry (mainly for debug purposes), default value of false
 */
class UGContourRingDetector(
        // primary constructor
        private var hardwareMap: HardwareMap, // hardwareMap variable being passed into the detector. no default value
        private var cameraDirection: OpenCvInternalCamera.CameraDirection = OpenCvInternalCamera.CameraDirection.BACK,
        private val telemetry: Telemetry? = null,
        private var debug: Boolean = false,
) {
    /**
     * Companion Object to hold all Configuration needed for the Pipeline
     *
     * @see UGContourRingPipe.Config
     */
    companion object PipelineConfiguration {
        /** Width and Height of the camera **/
        var CAMERA_WIDTH = 320
        var CAMERA_HEIGHT = 240

        /** Horizon value in use, anything above this value (less than the value) since
         * (0, 0) is the top left of the camera frame **/
        var HORIZON = 0

        /** Value storing whether or not the orientation of the camera is in portrait mode **/
        var IS_PORTRAIT_MODE = false

        /** Value storing the orientation of the camera **/
        var CAMERA_ORIENTATION: OpenCvCameraRotation = OpenCvCameraRotation.UPSIDE_DOWN
    }

    // camera variable, lateinit, initialized in init() function
    lateinit var camera: OpenCvCamera

    // flag to tell init() function whether or not to set up webcam
    private var isUsingWebcam = false

    // name of webcam, lateinit, initialized if webcam constructor is called otherwise will error is accessed
    private lateinit var webcamName: String

    // pipeline, lateinit, initialized in the init() function after creating of UGContourPipeline instance
    private lateinit var ftcLibPipe: UGContourRingPipe

    /**
     * @param hMap hardwareMap
     * @param webcamName name of webcam in use
     */
    constructor(hMap: HardwareMap, webcamName: String) : this(hardwareMap = hMap) {
        this.webcamName = webcamName
        this.isUsingWebcam = true
    }

    /**
     * @param hMap hardwareMap
     * @param webcamName name of webcam in use
     * @param telemetry If wanted, provide a telemetry object to the constructor to get stacktrace if an
     * error occurs (mainly for debug purposes), default value of null
     * @param debug If true, all intermediate calculation results (except showing mat operations) will
     * be printed to telemetry (mainly for debug purposes), default value of false
     */
    constructor(hMap: HardwareMap, webcamName: String, telemetry: Telemetry, debug: Boolean) : this(hardwareMap = hMap, telemetry = telemetry, debug = debug) {
        this.webcamName = webcamName
        this.isUsingWebcam = true
    }

    // returns the height detected by the pipeline
    val height: UGContourRingPipe.Height
        get() = ftcLibPipe.height

    // init function to initialize camera and pipeline
    fun init() {
        val cameraMonitorViewId = hardwareMap
                .appContext
                .resources
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.packageName,
                )
        camera = if (isUsingWebcam) {
            OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(
                            hardwareMap.get(
                                    WebcamName::class.java,
                                    webcamName
                            ),
                            cameraMonitorViewId,
                    )
        } else {
            OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(
                            cameraDirection, cameraMonitorViewId,
                    )
        }

        camera.setPipeline(
                UGContourRingPipe(
                        telemetry = telemetry,
                        debug = debug,
                ).apply {
                    UGContourRingPipe.CAMERA_WIDTH = if (IS_PORTRAIT_MODE) CAMERA_HEIGHT else CAMERA_WIDTH
                    UGContourRingPipe.HORIZON = HORIZON
                    ftcLibPipe = this
                }
        )
        lowerOrange = Scalar(0.0, 147.0, 0.0)
        upperOrange = Scalar(255.0, 189.0, 120.0)
        camera.openCameraDeviceAsync {
            camera.openCameraDevice()
            camera.startStreaming(
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    CAMERA_ORIENTATION,
            )
        }
    }

}