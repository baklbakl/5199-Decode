package org.ducktapeandaprayer.ftc.vision;

import android.hardware.usb.UsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerImpl;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.CachingExposureControl;
import org.firstinspires.ftc.robotcore.internal.collections.MutableReference;
import org.opencv.core.Mat;
import org.openftc.easyopencv.*;

import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public abstract class PipelineStarter extends OpenCvPipeline {
    public static int gain = 180; // Min 0, max 255
    public static long exposure = 7; // Max 204, min 0.
    public static int framesBeforeSet = 8; // Max 204, min 0.
    public static int framesBeforeCheck = 8; // Max 204, min 0.

    public final OpenCvWebcam camera;

    private CameraAttachmentCallback cameraCallback;

    private class CameraAttachmentCallback implements CameraManagerInternal.UsbAttachmentCallback {
        private final SerialNumber ourSerialNumber;
        private final WeakReference<CachingExposureControl> exposureControl;
        public volatile boolean settingsUpdateNeeded;

        public CameraAttachmentCallback(SerialNumber ourSerialNumber, CachingExposureControl exposureControl) {
            this.ourSerialNumber = ourSerialNumber;
            this.exposureControl = new WeakReference<>(exposureControl);
        }

        @Override
        public void onAttached(UsbDevice usbDevice, SerialNumber serialNumber, MutableReference<Boolean> claimed) {
            RobotLog.ii("Pipeline starter", "Camera Connected with serial number: \"" + serialNumber.toString() + "\"");
        }

        @Override
        public void onDetached(UsbDevice usbDevice) {
            RobotLog.ii("Pipeline starter", "Camera disconnected with serial number: \"" + usbDevice.getSerialNumber() + "\" Our camera: " + ourSerialNumber);
            if(ourSerialNumber.equals(usbDevice.getSerialNumber())) {
                RobotLog.ii("Pipeline starter", "Our camera was disconnected");
                CachingExposureControl exposureControlLocal = exposureControl.get();
                if(exposureControlLocal == null) {
                    RobotLog.ii("PipelineStarter", "Exposure Control cleaned up while we are still taking callbacks");
                } else {
                    try {
                        Field lockField = exposureControlLocal.getClass().getDeclaredField("lock");
                        lockField.setAccessible(true);
                        lockField.get(exposureControlLocal);
                        synchronized (lockField) {
                            Field modeField = exposureControlLocal.getClass().getDeclaredField("Mode");
                            modeField.setAccessible(true);
                            modeField.set(exposureControlLocal, null);

                            Field aePriorityField = exposureControlLocal.getClass().getDeclaredField("aePriority");
                            aePriorityField.setAccessible(true);
                            aePriorityField.set(exposureControlLocal, null);

                            Field nsExposureField = exposureControlLocal.getClass().getDeclaredField("nsExposure");
                            nsExposureField.setAccessible(true);
                            nsExposureField.setLong(exposureControlLocal, ExposureControl.unknownExposure);
                        }
                    } catch (NoSuchFieldException | IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                }

                settingsUpdateNeeded = true;

            }
        }
    }

    public PipelineStarter(MainBotHardware hardware, int width, int height) {
        int cameraMonitorViewId = hardware.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.hardwareMap.appContext.getPackageName());
        if(hardware.webcamName == null) {
            throw new RuntimeException("Pipeline Starter / Vision cannot be started without a camera");
        }
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardware.webcamName, cameraMonitorViewId);
        try {
            CameraManagerImpl cameraManager = (CameraManagerImpl) ClassFactory.getInstance().getCameraManager();
            Field callbacksField = cameraManager.getClass().getDeclaredField("callbacks");
            callbacksField.setAccessible(true);
            ArrayList<CameraManagerInternal.UsbAttachmentCallback> callbacks = (ArrayList<CameraManagerInternal.UsbAttachmentCallback>) callbacksField.get(cameraManager);
            CameraAttachmentCallback callBackToRemove = null;

            synchronized (callbacks) {
                for(CameraManagerInternal.UsbAttachmentCallback callback : callbacks) {
                    if(callback instanceof CameraAttachmentCallback) {
                        if(callBackToRemove == null) {
                            callBackToRemove = (CameraAttachmentCallback) callback;
                        } else {
                            throw new RuntimeException("Should not have to remove more than one camera callback to remove");
                        }

                    }
                }

                if(callBackToRemove != null) {
                    RobotLog.ii("PipelineStarter", "Removing Callback with serial number \"" + callBackToRemove.ourSerialNumber + "\"");
                    cameraManager.unregisterReceiver(callBackToRemove);
                }

                cameraManager.registerReceiver(new CameraAttachmentCallback(hardware.webcamName.getSerialNumber()));
            }
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.YUY2);
                //updateCameraSettings();
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public PipelineStarter(MainBotHardware hardware) {
        this(hardware, 800, 448);
    }

    private boolean checkCameraSettings() {
        return camera.getExposureControl().getMode() != ExposureControl.Mode.Manual || camera.getGainControl().getGain() != gain || camera.getExposureControl().getExposure(TimeUnit.MILLISECONDS) != exposure;
    }

    private void updateCameraSettings() {
        if(camera.getExposureControl().getMode() != ExposureControl.Mode.Manual) {
            RobotLog.ii("PipelineStarter", "mode: " + camera.getExposureControl().getMode());
            camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
        }

        if(camera.getGainControl().getGain() != gain) {
            RobotLog.ii("PipelineStarter", "gain: " + camera.getGainControl().getGain() + " " + gain);
            camera.getGainControl().setGain(gain);
        }

        if(camera.getExposureControl().getExposure(TimeUnit.MILLISECONDS) != exposure) {
            RobotLog.ii("PipelineStarter", "exposure: " + camera.getExposureControl().getExposure(TimeUnit.MILLISECONDS) + " " + exposure);
            camera.getExposureControl().setExposure(exposure, TimeUnit.MILLISECONDS);
        }
    }

    public String getTimingTelemetry() {
        StringBuilder sb = new StringBuilder();
        sb.append("FPS: ").append(camera.getFps()).append('\n');
        sb.append("Ms per frame processing: ").append(camera.getPipelineTimeMs()).append('\n');
        sb.append("Ms per frame overhead: ").append(camera.getOverheadTimeMs()).append('\n');
        sb.append("Ms per frame total: ").append(camera.getTotalFrameTimeMs());
        return sb.toString();
    }

    private int frameCountTillSet = 0;
    private int frameCountTillCheck = 0;

    //TODO: Document why we apply the settings like this ...
    @Override
    public Mat processFrame(Mat mat) {
        if(frameCountTillSet < framesBeforeSet) {
            frameCountTillSet++;
        } else if(frameCountTillSet == framesBeforeSet) {
            frameCountTillSet++;
            updateCameraSettings();
        }

        if(frameCountTillSet == framesBeforeSet + 1) {
            if(frameCountTillCheck < framesBeforeCheck) {
                frameCountTillCheck++;
            } else if(frameCountTillCheck == framesBeforeCheck) {
                frameCountTillCheck = 0;
                if(checkCameraSettings()) {
                    frameCountTillSet = framesBeforeSet;
                }
            }
        }
        return childProcessFrame(mat);
    }

    public abstract Mat childProcessFrame(Mat mat);
}
