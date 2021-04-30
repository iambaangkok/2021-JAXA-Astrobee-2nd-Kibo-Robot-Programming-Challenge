package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import static android.content.ContentValues.TAG;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();

        // astrobee is undocked and the mission starts
        // move(11.71, -9.53, 5.35, 0, 0, 0, 1);
        Point point = new Point(11.21, -9.8, 4.79);
        Quaternion quaternion = new Quaternion(0f, 0f,-0.707f, 0.707f);

        api.moveTo(point, quaternion, true);

        String contents = readQR();
        api.sendDiscoveredQR(contents);

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

    public void move(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public Mat undistortPic(Mat src , double[][] NavCam){
        Mat dst = new Mat(1280,960,CvType.CV_8UC1);
        double[] camera = NavCam[0];
        double[] distCoe = NavCam[1];

        Mat cameraMat = new Mat(3,3, CvType.CV_64F);
        cameraMat.put(0,0,camera);

        Mat distCoeMat = new Mat(1,5,CvType.CV_64F );
        distCoeMat.put(0,0,distCoe);

        Imgproc.undistort(src , dst , cameraMat , distCoeMat);

        return dst;
    }

    public String readQR(){
        String content = "";
        Mat pic = undistortPic(api.getMatNavCam() , api.getNavCamIntrinsics());
        QRCodeDetector detector = new QRCodeDetector();
        content = detector.detectAndDecode(pic);
        int loopCounter = 0;
        final int LOOP_MAX = 5;
        while( content.isEmpty() && loopCounter < LOOP_MAX){
            content = detector.detectAndDecode(pic);
            loopCounter++;
        }
        Log.i(TAG, "readQR: " + content);
        return content;
    }
}

