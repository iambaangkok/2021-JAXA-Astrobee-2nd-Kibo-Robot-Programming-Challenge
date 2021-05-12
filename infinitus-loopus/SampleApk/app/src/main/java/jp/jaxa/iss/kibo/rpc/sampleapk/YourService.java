package jp.jaxa.iss.kibo.rpc.sampleapk;

// astrobee library
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// android library
import android.os.SystemClock;
import android.util.Log;
import android.graphics.Bitmap;
// zxing library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.DecodeHintType;
import com.google.zxing.BarcodeFormat;
// opencv library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;
import org.opencv.calib3d.Calib3d;
// java library
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Arrays;
import java.util.Map;
import java.util.List;
import javax.vecmath.Vector3f;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 3;
    final int NAV_CAM_WIDTH = 1280;
    final int NAV_CAM_HEIGHT = 960;
    final Point pointA = new Point(11.21, -9.8, 4.79);
    static long startTime = 0;
    @Override
    protected void runPlan1(){
        final String TAG = "[main]: ";

        api.startMission();
        startTime = getTime();

        // move to point A (11.21, -9.8, 4.79) quaternion A (0, 0, -0.707f, 0.707f)
        moveTo(11.21, -9.8, 4.79, 0, 0, -0.707f, 0.707f);

        // scan QR Code to get point A' (qrData[0], qrData[1], qrData[2]) quaternion A' (0, 0, -0.707, 0.707) KOZ pattern (qrData[3])
        float[] qrData = qrEvent();
        int kozPattern = (int)qrData[0];

        // move to point A' (11.05, -9.80, 5.51) quaternion A (0, 0, -0.707f, 0.707f)  // delta pos = (-0.16, 0, +0.72)
        Point p60 = new Point();
        if(kozPattern == 2){
            p60 = averagePoint(pointA, new Point(qrData[1],qrData[2],qrData[3]), 60);
            moveTo(p60.getX(),p60.getY(),p60.getZ(), 0, 0, -0.707f, 0.707f);
        }

        double[] targetPoint = arEvent();

        Point robotPos = getRobotPosition();

        Vector3f forward = new Vector3f((float)(targetPoint[0]-robotPos.getX()), (float)(targetPoint[1]-robotPos.getY()), (float)(targetPoint[2]-robotPos.getZ()));
        Vector3f up = new Vector3f(0,0,-1);

        Quaternion lookAtTarget = quaternionLookRotation(forward,up);

        moveTo(robotPos.getX(),robotPos.getY(),robotPos.getZ(), lookAtTarget.getX(), lookAtTarget.getY(), lookAtTarget.getZ(), lookAtTarget.getW());
        LogT(TAG,"laser");
        api.laserControl(true);
        LogT(TAG,"snap");
        api.takeSnapshot();

        LogT(TAG, "successful");

        api.reportMissionCompletion();
    }


    private static long getTime(){
        return SystemClock.elapsedRealtime();
    }
    private static long getElapsedTime(){
        return getTime()-startTime;
    }
    private static String getElapsedTimeS(){ return " _Time_: " + String.valueOf(getElapsedTime()); }
    private static void LogT(String TAG, String msg){
        Log.d(TAG,msg + " " + getElapsedTimeS());
    }

    private Point getRobotPosition(){
        final String TAG = "[getRobotPosition]: ";
        LogT(TAG,"start");

        Kinematics data = api.getTrustedRobotKinematics();
        Point p = data.getPosition();

        LogT(TAG, "robot position = " + p.toString());

        return p;
    }
    private Quaternion getRobotOrientation(){
        final String TAG = "[getRobotRotation]: ";
        LogT(TAG,"start");

        Kinematics data = api.getTrustedRobotKinematics();

        Quaternion q = data.getOrientation();

        LogT(TAG, "robot position = " + q.toString());

        return q;
    }
    private PointCloud getPointCloud(){ return api.getPointCloudHazCam(); }

    private Point averagePoint(Point from, Point to, double percent){
        final String TAG = "[averagePoint]: ";
        LogT(TAG,"start");

        double x = from.getX() + (to.getX()-from.getX())*percent/100;
        double y = from.getY() + (to.getY()-from.getY())*percent/100;
        double z = from.getZ() + (to.getZ()-from.getZ())*percent/100;

        Point avg = new Point(x,y,z);
        LogT(TAG,"avg = " + avg.toString());

        return avg;
    }

    private void moveTo(double x, double y, double z, float qx, float qy, float qz, float qw){
        final String TAG = "[moveTo]: ";

        Point p = new Point(x,y,z);
        Quaternion q = new Quaternion(qx,qy,qz,qw);
        int loopCount = 0;
        Result result;
        LogT(TAG, "start");

        do {
            result = api.moveTo(p,q,true);
            loopCount++;
        } while(!result.hasSucceeded() && loopCount < LOOP_MAX);

        LogT(TAG,"finished");

    }
    private void setFlashOn(boolean status, int brightnessIndex){
        final String TAG = "[setFlashOn]: ";

        float brightness = 0.33f;
        brightness *= brightnessIndex;

        LogT(TAG, "start");
        if(status == true){
            api.flashlightControlFront(brightness);
            LogT(TAG, "brightness = " + String.valueOf(brightness));
            try{
                LogT(TAG, "sleeping");
                Thread.sleep(1000);
            }catch (InterruptedException e){
                LogT(TAG, "interrupted");
                e.printStackTrace();
            }
        }else{
            api.flashlightControlFront(0f);
            LogT(TAG, "off");
        }
        LogT(TAG, "done");
        return;
    }

    private Rect cropImage(double xPercent, double yPercent,double cropPercent){
        final String TAG = "[cropImage]: ";

        LogT(TAG,"start");

        final int WIDTH = 1280;
        final int HEIGHT = 960;

        int x = (int)(xPercent/100*WIDTH);
        int y = (int)(yPercent/100*HEIGHT);
        int width = (int)(cropPercent/100*WIDTH);
        int height = (int)(cropPercent/100*HEIGHT);

        LogT(TAG,"done");

        return new Rect(x,y,width,height);
    }
    private BinaryBitmap getNavCamImage(){
        final String TAG = "[getNavCamImage]: ";

        LogT(TAG, "start");

        Mat image = new Mat(api.getMatNavCam(), cropImage(45, 50, 32));

        Bitmap bitmap = Bitmap.createBitmap(image.width(), image.height(), Bitmap.Config.ARGB_8888);
        matToBitmap(image, bitmap);

        int[] intArray = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), intArray);
        BinaryBitmap binarizedBitmap = new BinaryBitmap(new HybridBinarizer(source));

        LogT(TAG, "done");

        return binarizedBitmap;
    }

    private Mat undistort(Mat sourceImage, Rect roi){
        final String TAG = "[undistortImage]: ";
        LogT(TAG, "start");
        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        LogT(TAG, "got navCamIntrinsics " + Arrays.deepToString(navCamIntrinsics));

        int row = 0, col = 0;
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        cameraMat.put(row, col, navCamIntrinsics[0]);
        LogT(TAG, "cameraMat put " + cameraMat.toString() + " " + cameraMat.dump());

        row = 0; col = 0;
        Mat distortionCoeff = new Mat(1, 5, CvType.CV_32FC1);
        distortionCoeff.put(row,col,navCamIntrinsics[1]);
        LogT(TAG, "distortCoeff put " + distortionCoeff.toString() + " " + distortionCoeff.dump());

        Mat undistortedPic = new Mat(1280, 960, CvType.CV_8UC1);

        LogT(TAG, sourceImage.size().toString());
        LogT(TAG,  cameraMat.size().toString());
        LogT(TAG,  distortionCoeff.size().toString());
        Size sourceImageSize = sourceImage.size();

        Mat newCameraMat = Calib3d.getOptimalNewCameraMatrix(cameraMat, distortionCoeff, sourceImageSize, 1, sourceImageSize, roi);
        LogT(TAG, "got optimalNewCamMat " + newCameraMat.dump());
        LogT(TAG, newCameraMat.size().toString());

        Imgproc.undistort(sourceImage,undistortedPic,cameraMat,distortionCoeff,newCameraMat);
        LogT(TAG, "finished undistort");

        return undistortedPic;
    }
    private float[] getQRDataContent(String qrData){
        final String TAG = "[getQRDataContent] :";
        LogT(TAG, "start");

        String[] multi_contents = qrData.split(",");
        int kozPattern = Integer.parseInt(multi_contents[0].substring(5));
        float x = Float.parseFloat(multi_contents[1].substring(4));
        float y = Float.parseFloat(multi_contents[2].substring(4));
        float z = Float.parseFloat(multi_contents[3].substring(4, multi_contents[3].length()-1));
        LogT(TAG, "contents: " + kozPattern + " " + x + " " + y + " " + z);
        LogT(TAG, "finished");

        return new float[] {kozPattern, x, y, z};
    }
    private float [] qrEvent(){
        final String TAG = "[qrEvent]: ";
        LogT(TAG,"start");

        String qrData = null;
        int loopCount = 0;
        BinaryBitmap bitmap;
        Map<DecodeHintType, Object> hints = new Hashtable<>();
        hints.put(DecodeHintType.TRY_HARDER, "");
        List<BarcodeFormat> qr = new ArrayList<>(); qr.add(BarcodeFormat.QR_CODE);
        hints.put(DecodeHintType.POSSIBLE_FORMATS, qr);
        hints.put(DecodeHintType.CHARACTER_SET, "utf-8");

        while(qrData == null && loopCount < LOOP_MAX){
            LogT(TAG, "loopCount " + loopCount);
            setFlashOn(true,loopCount);
            try{
                LogT(TAG, "reading qr code");
                bitmap = getNavCamImage();
                QRCodeReader reader = new QRCodeReader();
                com.google.zxing.Result result = reader.decode(bitmap,hints);
                qrData = result.getText();
                if(qrData != null){
                    api.sendDiscoveredQR(qrData);
                    LogT(TAG, "data : " + qrData);
                    LogT(TAG, "qr code read successfully");

                    float[] content = getQRDataContent(qrData);
                    setFlashOn(false,0);
                    return content;
                }
            }catch(Exception e){
                LogT(TAG, "an error occurred while reading qr code");
                e.printStackTrace();
            }
            setFlashOn(false,0);
            loopCount++;
        }
        setFlashOn(false,0);

        LogT(TAG, "failed to read qr code");
        return null;
    }

    private static Quaternion quaternionLookRotation(Vector3f forward, Vector3f up){
        final String TAG = "[quaternionLookRotation]: ";
        LogT(TAG,"start");
        forward.normalize();

        Vector3f vector = forward;
        Vector3f vector2 = new Vector3f();
        vector2.cross(up,vector); vector2.normalize();
        Vector3f vector3 = new Vector3f();
        vector3.cross(vector,vector2);
        float m00 = vector2.x;
        float m01 = vector2.y;
        float m02 = vector2.z;
        float m10 = vector3.x;
        float m11 = vector3.y;
        float m12 = vector3.z;
        float m20 = vector.x;
        float m21 = vector.y;
        float m22 = vector.z;


        float num8 = (m00 + m11) + m22;

        if (num8 > 0f){
            float num = (float)Math.sqrt(num8 + 1f);
            float nw = num * 0.5f;
            num = 0.5f / num;
            float nx = (m12 - m21) * num;
            float ny = (m20 - m02) * num;
            float nz = (m01 - m10) * num;
            Quaternion quaternion = new Quaternion(nx,ny,nz,nw);
            LogT(TAG,"result = " + quaternion.toString());
            return quaternion;
        }
        if ((m00 >= m11) && (m00 >= m22)){
            float num7 = (float)Math.sqrt(((1f + m00) - m11) - m22);
            float num4 = 0.5f / num7;
            float nx = 0.5f * num7;
            float ny = (m01 + m10) * num4;
            float nz = (m02 + m20) * num4;
            float nw = (m12 - m21) * num4;
            Quaternion quaternion = new Quaternion(nx,ny,nz,nw);
            LogT(TAG,"result = " + quaternion.toString());
            return quaternion;
        }
        if (m11 > m22){
            float num6 = (float)Math.sqrt(((1f + m11) - m00) - m22);
            float num3 = 0.5f / num6;
            float nx = (m10+ m01) * num3;
            float ny = 0.5f * num6;
            float nz = (m21 + m12) * num3;
            float nw = (m20 - m02) * num3;
            Quaternion quaternion = new Quaternion(nx,ny,nz,nw);
            LogT(TAG,"result = " + quaternion.toString());
            return quaternion;
        }
        float num5 = (float)Math.sqrt(((1f + m22) - m00) - m11);
        float num2 = 0.5f / num5;
        float nx = (m20 + m02) * num2;
        float ny = (m21 + m12) * num2;
        float nz = 0.5f * num5;
        float nw = (m01 - m10) * num2;
        Quaternion quaternion = new Quaternion(nx,ny,nz,nw);
        LogT(TAG,"result = " + quaternion.toString());
        return quaternion;
    }

    private double[] getMarkerCenter(Mat corner){
        final String TAG = "[getMarkerCenter]: ";
        LogT(TAG,"start");

        double[] sum = new double[2];
        sum[0] = sum[1] = 0;
        LogT(TAG,"init");
        for(int j = 0 ; j < 4; ++j){
            LogT(TAG,"" + j + "x");
            sum[0] += corner.get(0,2*j)[0];
            LogT(TAG,"" + j + "y");
            sum[1] += corner.get(0,2*j+1)[0];
        }
        sum[0] /= 4;
        sum[1] /= 4;
        LogT(TAG,"corner_center = (" + sum[0] + "," + sum[1] + ")");

        return sum;
    }
    private double[] getARCenter(List<Mat> corners){
        final String TAG = "[getARCenter]: ";
        LogT(TAG,"start");

        double centerX = 0, centerY = 0;

        for(int i = 0 ; i < 4; ++i){
            double[] sumXY = getMarkerCenter(corners.get(i));
            centerX += sumXY[0];
            centerY += sumXY[1];
        }
        centerX /= 4;
        centerY /= 4;
        double[] center = new double[2];
        center[0] = centerX; center[1] = centerY;

        LogT(TAG,"ar center = (" + centerX + "," + centerY + ")");
        return center;
    }
    private double[] getARCenterPoint(double[] pointFromNavCam){
        final String TAG = "[getARCenterPoint]: ";
        LogT(TAG,"start");

        PointCloud pointCloud = getPointCloud();
        if(pointCloud != null){
            int pointCloudWidth = pointCloud.getWidth(); // 224 : 1280
            int pointCloudHeight = pointCloud.getHeight(); // 171 : 960
            Point[] pointArray = pointCloud.getPointArray();

            int pointCloudX = (int)(pointFromNavCam[0]/NAV_CAM_WIDTH*pointCloudWidth);
            int pointCloudY = (int)(pointFromNavCam[1]/NAV_CAM_HEIGHT*pointCloudHeight);
            Point targetPoint = pointArray[pointCloudY*pointCloudWidth + pointCloudX];
            LogT(TAG,  "targetPoint = " + targetPoint.toString());
            double[] result = new double[3];
            result[0] = targetPoint.getX();
            result[1] = targetPoint.getY();
            result[2] = targetPoint.getZ();
            return result;
        }else{
            LogT(TAG, "failed to get point cloud");
            return null;
        }
    }

    public double[] arEvent(){
        final String TAG = "[arEvent]: ";

        int arContent = 0;
        int loopCount = 0;
        double[] result = new double[3];

        Rect roi = new Rect();
        Mat image = undistort(api.getMatNavCam(),roi);
        Mat ids = new Mat();
        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();

        LogT(TAG, "start");

        while (arContent == 0 && loopCount < LOOP_MAX){
            LogT(TAG, "loopCount " + loopCount);
            try{
                LogT(TAG, "reading ar");
                Aruco.detectMarkers(image, dict, corners, ids);
                LogT(TAG, "ids " + ids.dump());
                for (int i = 0; i < corners.size(); i++) {
                    LogT(TAG, "corners[" + i + "]=" + corners.get(i).dump());
                }

                double[] arCenter = getARCenter(corners);
                result = getARCenterPoint(arCenter);

                arContent = (int) ids.get(0, 0)[0];

                LogT(TAG,"ar read successfully");
            }
            catch (Exception e)
            {
                LogT(TAG,"an error occurred while reading ar");
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            LogT(TAG, "failed to read ar");
            long stop_time = SystemClock.elapsedRealtime();

            loopCount++;
        }
        return result;
    }



}

