package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionTensorflow {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//    String[] LABELS = {"redProp", "blueProp"};
    String[] LABELS = {"Pixel"};

    public VisionTensorflow(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.telemetry.addData("status:", "works");
        this.telemetry.update();
        initTfod();
    }

    public void detectProp() {
        telemetryTfod(); //run detection

        // Push telemetry to the Driver Station.
        telemetry.update();
    }

    public void stopDetecting() {
        visionPortal.close();
    }


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        telemetry.addData("status:", "works2");
        telemetry.update();

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName("CenterStage.tflite")
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
//
                .build();

        telemetry.addData("status:", "works3");
        telemetry.update();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        telemetry.addData("status:", "works4");
        telemetry.update();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        telemetry.addData("status:", "works5");
        telemetry.update();

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(960, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

//         Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.30f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);


    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}
