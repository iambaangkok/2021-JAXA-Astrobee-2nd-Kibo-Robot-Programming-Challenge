package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d.*;
import org.opencv.core.Size;

import java.nio.ByteBuffer;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();

        // astrobee is undocked and the mission starts
        moveToWrapper(11.71, -9.53, 5.35, 0, 0, 0, 1);
        String contents = QR();
        // irradiate the laser
        api.laserControl(true);

        // take snapshots9
        api.takeSnapshot();

        // move to the rear of Bay7
        moveToWrapper(10.275, -10.314, 4.295, 0, -0.7071068, 0, 0.7071068);

        // Send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // You can add your method

    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private String QR(){

        Mat pic = api.getMatNavCam();
        Size size = pic.size();
        double[][] NavCam = api.getNavCamIntrinsics();
        double[] camera = NavCam[0];
        double[] distCoe = NavCam[1];

        ByteBuffer bcam = ByteBuffer.allocate(camera.length * 8);
        for(double d : camera) {
            bcam.putDouble(d);
        }

        ByteBuffer bdist = ByteBuffer.allocate(distCoe.length * 8);
        for(double d : distCoe) {
            bdist.putDouble(d);
        }

        Mat cameraMat = new Mat(1,9,6,bcam);
        cameraMat.reshape(3,3);

        Mat distCoeMat = new Mat(1,5,6 , bdist);

        Mat newCameraMat = Calib3d.getOptimalNewCameraMatrix(cameraMat , distCoeMat , size , 1 ,size);

        Mat dst = new Mat();
        Calib3d.fisheye_undistortImage(pic , dst , cameraMat , distCoeMat , newCameraMat);

        return "";


    }

}

