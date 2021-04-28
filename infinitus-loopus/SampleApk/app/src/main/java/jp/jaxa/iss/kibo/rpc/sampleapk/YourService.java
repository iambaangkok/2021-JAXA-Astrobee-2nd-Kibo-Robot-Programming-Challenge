package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// astrobee library
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
// android library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
// zxing library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;
import org.opencv.calib3d.Calib3d;
// opencv library
import java.util.ArrayList;
import java.util.List;
// java library


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 5;

    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();

        // move to point A
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);

        // scan QR Code to get point A'
        

        api.reportMissionCompletion();
    }

    private void moveToWrapper(double px, double py, double pz,
                               double qx, double qy, double qz,
                               double qw){

        final int LOOP_MAX = 5;
        final Point point = new Point(px, py, pz);
        final Quaternion quaternion = new Quaternion((float)qx, (float)qy,
                                                     (float)qz, (float)qw);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private void readQRCode(double px, double py, double pz, double qx, double qy, double qz, double qw, int noqr){

        // stabilize
        moveToWrapper(px, py, pz, qx, qy, qz, qw);

        // get nav cam pic
        Mat pic = api.getMatNavCam();

        // undistort fisheye
        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        Mat cameraMat = new Mat();
        cameraMat.put(3, 3, navCamIntrinsics[0]);
        Mat distortionCoeff = new Mat();
        distortionCoeff.put(1,5,navCamIntrinsics[1]);

        Rect roi = new Rect();

        Mat newCameraMat = Calib3d.getOptimalNewCameraMatrix(cameraMat,distortionCoeff,pic.size(),1, pic.size(), roi);

        Mat undistortedPic = new Mat();
        Calib3d.fisheye_undistortImage(pic,undistortedPic,cameraMat,distortionCoeff,newCameraMat);

        //Bitmap  bmap = new BinaryBitmap (pic);

    }




}

