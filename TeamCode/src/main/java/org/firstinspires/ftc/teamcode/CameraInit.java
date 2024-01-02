package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvCamera;


public class CameraInit extends e {
    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
        @Override
        public void onOpened()
        {
            // Usually this is where you'll want to start streaming from the camera (see section 4)
        }
        @Override
        public void onError(int errorCode)
        {
            /*
             * This will be called if the camera could not be opened
             */
        }
    });
}
