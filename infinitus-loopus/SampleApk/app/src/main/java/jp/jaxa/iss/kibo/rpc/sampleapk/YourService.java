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
import javax.vecmath.Quat4f;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 2;
    final int NAV_CAM_WIDTH = 1280;
    final int NAV_CAM_HEIGHT = 960;
    final Point pointA = new Point(11.21, -9.8, 4.79);
    final Quaternion quaternionA = new Quaternion(0, 0, -0.707f, 0.707f);
    final Vector3f up = new Vector3f(0,-1,0);

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

        //Quaternion lookAToAPrime = quaternionLookRotation(new Vector3f(11.05f - 11.21f, -9.8f + 9.8f, 5.51f - 4.79f), up);
        //moveTo(11.21, -9.8, 4.79, lookAToAPrime);

        Point p60 = pointA;
        Point p60yminus = pointA;
        Quaternion lookTowardsAR = new Quaternion(1,0,0,0);
        if(kozPattern == 2){
            // look a bit down
            p60 = averagePoint(pointA, new Point(qrData[1],qrData[2],qrData[3]), 60);
            p60yminus = offsetPoint(p60,0,-0.2,0);
            lookTowardsAR = quaternionRelativeRotate(quaternionA, new Vector3f(0,1,0), -35);
            moveTo(p60yminus, lookTowardsAR);
        }
        else if(kozPattern == 8){
            // move down and turn left
            p60 = new Point(qrData[1] , qrData[2], qrData[3]);
            p60yminus = offsetPoint(p60,0.22,-0.2,0);
            lookTowardsAR = quaternionRelativeRotate(quaternionA, new Vector3f(0,0,1), -30);
            moveTo(p60yminus, lookTowardsAR);
        }

        // read AR
        Quaternion looking = lookTowardsAR;
        double[] arData = new double[6]; // angle offset x, y, angle threshold, pixel offset x, y, distancePerPixel

        arData[2] = 0;
        arData = arEvent(10);

        // determine whether to aim by turning or moving
        Point pAimAR = p60yminus;


        LogT(TAG,"aim by moving towards target2");
        double[] eulers = quaternionToEulers(looking);
        double total_y = (-arData[4]) * arData[5];
        double total_x = (-arData[3]) * arData[5];

        double dx = Math.cos(eulers[2] + 90) * Math.cos(eulers[0]) * (total_y);
        double dy = Math.cos(eulers[2] + 90) * Math.sin(eulers[0]) * total_y;
        double dz = Math.sin(eulers[0]) * (total_y);

        double dx2 = Math.sin(eulers[2] + 90) * total_x;
        double dy2 = Math.cos(eulers[2]+ 90) * total_x;
        LogT(TAG, "dx dx2, dwy dy2, dz = " + dx + " " + dx2 + ", " + dy + " " + dy2 + ", " + dz);

        pAimAR = offsetPoint(p60yminus,dx + dx2 , 0, dz);

        moveTo(pAimAR, looking);
        wait(5000);

            /*double dx = -Math.sin(eulers[2]) * (arData[3]) * arData[5];

            double dy = Math.cos(eulers[2]) * Math.cos(eulers[1]) * (arData[4]) * arData[5];
            double dz = Math.sin(eulers[1]) * (arData[4]) * arData[5];*/



        // laser, snap, finish
        LogT(TAG,"laser on");
        api.laserControl(true);
        LogT(TAG,"snap");
        api.takeSnapshot();
        LogT(TAG,"laser off");
        api.laserControl(false);

        LogT(TAG, "going to pointB");
        goToB_event(kozPattern);

        LogT(TAG, "reporting mission completion");
        api.reportMissionCompletion();
        LogT(TAG, "successful");


    }

    //Utility
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
    private Point offsetPoint(Point start, double dx, double dy, double dz){
        final String TAG = "[offsetPoint]: ";
        LogT(TAG,"start");

        double x = start.getX() + dx;
        double y = start.getY() + dy;
        double z = start.getZ() + dz;

        Point offset = new Point(x,y,z);
        LogT(TAG,"offset = " + offset.toString());

        return offset;
    }
    private void moveTo(double x, double y, double z, float qx, float qy, float qz, float qw){
        final String TAG = "[moveTo]: ";

        Point p = new Point(x,y,z);
        Quaternion q = new Quaternion(qx,qy,qz,qw);
        int loopCount = 0;
        Result result;
        LogT(TAG, "start " + x + "," + y + "," + z + " | " + qx + "," + qy + "," + qz + "," + qw);

        do {
            result = api.moveTo(p,q,true);
            loopCount++;
        } while(!result.hasSucceeded() && loopCount < LOOP_MAX);

        LogT(TAG,"finished");
    }
    private void moveTo(double x, double y, double z, Quaternion q){
        final String TAG = "[moveTo]: ";

        float qx = q.getX();
        float qy = q.getY();
        float qz = q.getZ();
        float qw = q.getW();


        Point p = new Point(x,y,z);
        int loopCount = 0;
        Result result;
        LogT(TAG, "start " + x + "," + y + "," + z + " | " + qx + "," + qy + "," + qz + "," + qw);

        do {
            result = api.moveTo(p,q,true);
            loopCount++;
        } while(!result.hasSucceeded() && loopCount < LOOP_MAX);

        LogT(TAG,"finished");
    }
    private void moveTo(Point p, Quaternion q){
        final String TAG = "[moveTo]: ";

        double x = p.getX();
        double y = p.getY();
        double z = p.getZ();
        float qx = q.getX();
        float qy = q.getY();
        float qz = q.getZ();
        float qw = q.getW();


        int loopCount = 0;
        Result result;
        LogT(TAG, "start " + x + "," + y + "," + z + " | " + qx + "," + qy + "," + qz + "," + qw);

        do {
            result = api.moveTo(p,q,true);
            loopCount++;
        } while(!result.hasSucceeded() && loopCount < LOOP_MAX);

        LogT(TAG,"finished");

    }

    private void goToB_event(int koz){
        final String TAG = "[GoB]: ";
        Log.d(TAG, "start");
        /*if(koz == 8){
            Log.d(TAG, "Go to point A");
            moveTo(11.21, -9.8, 4.79, 0, 0, -0.707f, 0.707f);
        }*/
        Log.d(TAG, "Move to entrance");
        moveTo(10.505,-9.2, 4.5, 0, 0, -0.707f, 0.707f);

        Log.d(TAG, "Pass through KOZ");
        moveTo(10.505, -8, 4.5, 0, 0, -0.707f, 0.707f);

        Log.d(TAG, "Go to point B");
        moveTo(10.6, -8, 4.5, 0, 0, -0.707f, 0.707f);

        Log.d(TAG, "end");
        return;
    }

    private void setFlashOn(boolean status, int brightnessIndex){
        final String TAG = "[setFlashOn]: ";

        float brightness = 0.33f;
        brightness *= brightnessIndex;

        LogT(TAG, "start");
        if(status == true){
            api.flashlightControlFront(brightness);
            LogT(TAG, "brightness = " + String.valueOf(brightness));
            wait(500);
        }else{
            api.flashlightControlFront(0f);
            LogT(TAG, "off");
        }
        LogT(TAG, "done");
        return;
    }
    private void wait(int milliseconds){
        final String TAG = "[wait]: ";
        LogT(TAG, "start");

        try{
            LogT(TAG, "waiting for " + milliseconds + " ms");
            Thread.sleep(milliseconds);
        }catch (InterruptedException e){
            LogT(TAG, "interrupted");
            e.printStackTrace();
        }
        return;
    }

    //Image Processing
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

    private Mat undistortPoints(Mat points){
        final String TAG = "[undistortPoints]: ";
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

        Mat undistortedPoints = new Mat(points.rows(), points.cols(), points.type());

        LogT(TAG, points.size().toString());
        LogT(TAG,  cameraMat.size().toString());
        LogT(TAG,  distortionCoeff.size().toString());


        Imgproc.undistortPoints(points,undistortedPoints,cameraMat,distortionCoeff, new Mat(), cameraMat);
        LogT(TAG, "finished undistort points" + undistortedPoints.dump());

        return undistortedPoints;
    }

    //QR
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
            setFlashOn(true,(loopCount+1)%3);
            try{
                LogT(TAG, "reading qr code");
                wait(18000);
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

    //Quaternion
    private double[] quaternionToEulers(Quaternion q) {
        final String TAG = "[quaternionToEulers]: ";
        LogT(TAG,"start");
        double[] angles = new double[3]; // roll, pitch, yaw

        float qx = q.getX();
        float qy = q.getY();
        float qz = q.getZ();
        float qw = q.getW();

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        angles[0] = Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (qw * qy - qz * qx);
        if (Math.abs(sinp) >= 1){
            angles[1] = Math.copySign(Math.PI / 2, sinp); // use 90 degrees if out of range
        } else {
            angles[1] = Math.asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        angles[2] = Math.atan2(siny_cosp, cosy_cosp);

        LogT(TAG,"eulers = " + angles[0] + ", " + angles[1] + ", " + angles[2]);

        return angles;
    }
    private Quaternion eulersToQuaternion(double[] eulers){
        final String TAG = "[eulersToQuaternion]: ";
        LogT(TAG,"start");
        double roll = eulers[0];
        double pitch = eulers[1];
        double yaw = eulers[2];
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qw, qx, qy, qz;
        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;

        Quaternion q = new Quaternion((float)qx, (float)qy, (float)qz, (float)qw);
        LogT(TAG,"quaternion = " + q.toString());

        return q;
    }

    private double[] eulersDegToRad(double[] degs){
        final String TAG = "[eulersDegToRad]: ";
        LogT(TAG,"start");
        double[] rads = new double[3];
        for(int i = 0 ; i < 3; ++i){
            rads[i] = Math.toRadians(degs[i]);
        }
        LogT(TAG, "rads = " + rads[0] + ", " + rads[1] + ", " + rads[2]);
        return rads;
    }
    private double[] eulersRadToDeg(double[] rads){
        final String TAG = "[eulersRadToDeg]: ";
        LogT(TAG,"start");
        double[] degs = new double[3];
        for(int i = 0 ; i < 3; ++i){
            degs[i] = Math.toDegrees(rads[i]);
        }
        LogT(TAG, "deg = " + degs[0] + ", " + degs[1] + ", " + degs[2]);
        return degs;
    }

    private void logOrientationDetails(Quaternion q){
        eulersToQuaternion(eulersDegToRad(eulersRadToDeg(quaternionToEulers(q))));
        return;
    }

    private Quaternion createFromAxisAngle(float xx, float yy, float zz, float angle) {
        final String TAG = "[createFromAxisAngle]: ";
        LogT(TAG,"start");

        float aangle = (float)(angle*Math.PI/180);
        // Here we calculate the sin( theta / 2) once for optimization
        float factor = (float)Math.sin( aangle / 2.0f );

        // Calculate the x, y and z of the quaternion
        float x = xx * factor;
        float y = yy * factor;
        float z = zz * factor;

        // Calcualte the w value by cos( theta / 2 )
        float w = (float)Math.cos( aangle / 2.0f );
        Quat4f q = new Quat4f(x,y,z,w);
        q.normalize();
        Quaternion result = new Quaternion(q.x, q.y, q.z, q.w);
        LogT(TAG,"created quaternion = " + result.toString());
        return result;
    }
    private Quaternion multiplyQuaternion(Quaternion a, Quaternion b) {
        final String TAG = "[multiplyQuaternion]: ";
        LogT(TAG,"start");

        float ax = a.getX();
        float ay = a.getY();
        float az = a.getZ();
        float aw = a.getW();

        float bx = b.getX();
        float by = b.getY();
        float bz = b.getZ();
        float bw = b.getW();

        float x,y,z,w;
        Quat4f q1 = new Quat4f(ax,ay,az,aw);
        Quat4f q2 = new Quat4f(bx,by,bz,bw);

        q1.mul(q2);

        x = q1.x;
        y = q1.y;
        z = q1.z;
        w = q1.w;

        /*float x = aw * bx + ax * bw + ay * bz - az * by;    // i
        float y = aw * by - ax * bz + ay * bw + az * bx;    // j
        float z = aw * bz + ax * by - ay * bx + az * bw;    // k
        float w = aw * bw - ax * bx - ay * by - az * bz;    // 1*/
        Quaternion result = new Quaternion(x,y,z,w);
        LogT(TAG,"multiplied quaternion = " + result.toString());
        return result;
    }

    private Quaternion quaternionLookRotation(Vector3f forward, Vector3f up){
        final String TAG = "[quaternionLookRotation]: ";
        LogT(TAG,"start");

        Quaternion look = quaternionLookRotation0(forward, up);
        Quaternion rotAxis = createFromAxisAngle(0,1,0, -90);

        look = multiplyQuaternion(look,rotAxis);
        logOrientationDetails(look);

        LogT(TAG,"finished");

        return look;
    }
    private Quaternion quaternionLookRotation0(Vector3f forward, Vector3f up){
        final String TAG = "[quaternionLookRotation0]: ";
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

    private Quaternion quaternionRelativeRotate(Vector3f axis,float angle){
        final String TAG = "[relativeRotate]: ";
        LogT(TAG,"start");

        Quaternion look = getRobotOrientation();
        look = getRobotOrientation();
        Quaternion rotAxis = createFromAxisAngle(0,1,0, angle);

        look = multiplyQuaternion(look,rotAxis);
        logOrientationDetails(look);

        LogT(TAG,"finished");

        return look;
    }
    private Quaternion quaternionRelativeRotate(Quaternion startingQ, Vector3f axis,float angle){
        final String TAG = "[relativeRotate]: ";
        LogT(TAG,"start");

        Quaternion look = startingQ;
        Quaternion rotAxis = createFromAxisAngle(axis.x, axis.y, axis.z, angle);

        look = multiplyQuaternion(look,rotAxis);
        logOrientationDetails(look);

        LogT(TAG,"finished");

        return look;
    }

    //AR
    private double[] getMarkerCenter(Mat corner){
        final String TAG = "[getMarkerCenter]: ";
        LogT(TAG,"start");

        double[] sum = new double[2];
        sum[0] = sum[1] = 0;
        for(int j = 0 ; j < 4; ++j){
            LogT(TAG,"" + j + "x");
            sum[0] += corner.get(0,j)[0];
            LogT(TAG,"" + j + "y");
            sum[1] += corner.get(0,j)[1];
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

        double[] result = new double[3];
        /*if(pointFromNavCam[0] < 1280/2-556/2 || pointFromNavCam[0] > 1280/2+556/2){
            LogT(TAG,"x out of range");
            return result;
        }
        if(pointFromNavCam[1] < 1280/2-417/2 || pointFromNavCam[1] > 1280/2+417/2){
            LogT(TAG,"y out of range");
            return result;
        }*/

        // 0->556, 0->417


        int loopCount = 0;
        while(loopCount < LOOP_MAX){
            try{
                LogT(TAG,"getting point cloud hazcam");
                PointCloud pointCloud = api.getPointCloudHazCam();
                LogT(TAG,"get width");
                int pointCloudWidth = pointCloud.getWidth(); // 224 : 556
                LogT(TAG,"get height");
                int pointCloudHeight = pointCloud.getHeight(); // 171 : 417

                LogT(TAG,"get point array");
                Point[] pointArray = new Point[pointCloudWidth*pointCloudHeight];
                pointArray = pointCloud.getPointArray();

                LogT(TAG,"pcX");
                int pointCloudX = (int)(pointFromNavCam[0]/556*pointCloudWidth);
                LogT(TAG,"pcY");
                int pointCloudY = (int)(pointFromNavCam[1]/417*pointCloudHeight);

                LogT(TAG,"tgtPoint");
                Point targetPoint = pointArray[pointCloudY*pointCloudWidth + pointCloudX];
                LogT(TAG,  "targetPoint = " + targetPoint.toString());
                result[0] = targetPoint.getX(); // x
                result[1] = targetPoint.getY(); // y
                result[2] = targetPoint.getZ(); // z
                LogT(TAG,"ar center point retrieved");
                return result;
            }catch (Exception e){
                LogT(TAG, "an error occurred while getting ar center point");
                e.printStackTrace();
            }
            loopCount++;
        }
        LogT(TAG, "failed to get point cloud");
        return null;
    }

    private int[] getPixelOffsetFromCenter(double[] arCenter, double distanceThreshold){
        final String TAG = "[getPixelOffsetFromCenter]: ";
        LogT(TAG,"start");

        int[] result = new int[3];
        int centerX = NAV_CAM_WIDTH/2;
        int centerY = NAV_CAM_HEIGHT/2;

        result[0] = (int)arCenter[0]-centerX;
        result[1] = (int)arCenter[1]-centerY;

        double distanceFromCenter = Math.sqrt(result[0]*result[0] + result[1]*result[1]);
        LogT(TAG,"distance from center = " + distanceFromCenter);
        if(distanceFromCenter <= distanceThreshold){
            result[2] = 1;
        }else{
            result[2] = 0;
        }

        LogT(TAG, "pixel offset = " + result[0] + "," + result[1] + " : " + result[2]);

        return result;
    }
    private double[] pixelOffsetToAngleOffset(int[] pixelOffset, double angleThreshold){
        final String TAG = "[pixelOffsetToAngleOffset]: ";
        LogT(TAG,"start");

        double[] result = new double[3];

        double anglePerPixel = 130.0/Math.sqrt(Math.pow(NAV_CAM_WIDTH,2) + Math.pow(NAV_CAM_HEIGHT,2));

        LogT(TAG,"angle per pixel = " + anglePerPixel + "," + anglePerPixel);

        result[0] = pixelOffset[0]*anglePerPixel;
        result[1] = pixelOffset[1]*anglePerPixel;

        double angleFromCenter = Math.sqrt(result[0]*result[0] + result[1]*result[1]);
        LogT(TAG,"angle from center = " + angleFromCenter);
        if(angleFromCenter <= angleThreshold){ //Math.abs(result[0]) <= angleThreshold && Math.abs(result[1]) <= angleThreshold){
            result[2] = 1;
        }else{
            result[2] = 0;
        }

        LogT(TAG, "angle offset = " + result[0] + "," + result[1] + " : " + result[2]);

        return result;
    }

    private double getDistancePerPixel(List<Mat> corners){ // in meters
        final String TAG = "[getDistancePerPixel]: ";
        LogT(TAG,"start");

        double[][] markerCenters = new double[4][2];
        double distancePerPixel = 0;
        double leftMost = 10000;
        double rightMost = -10000;

        for(int i = 0 ; i < 4; ++i){
            markerCenters[i] = getMarkerCenter(corners.get(i));
            if(markerCenters[i][0] < leftMost){
                leftMost = markerCenters[i][0];
            }
            if(markerCenters[i][0] > rightMost){
                rightMost = markerCenters[i][0];
            }
        }
        double pixelDistance = rightMost-leftMost;
        LogT(TAG, "l r pd = " + leftMost  + ", " + rightMost + ", " + pixelDistance);

        distancePerPixel = (0.1125*2/pixelDistance);

        LogT(TAG,"distance(meters) per pixel = " + distancePerPixel);
        return distancePerPixel;
    }

    public double[] arEvent(double angleThreshold){ //returns turn angle around y, around z, is within angleThreshold, pixel offset x, y, distancePerPixel
        final String TAG = "[arEvent]: ";

        int arContent = 0;
        int loopCount = 0;
        double[] result = new double[6];

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        LogT(TAG, "start");

        while (arContent == 0 && loopCount < LOOP_MAX){
            LogT(TAG, "loopCount " + loopCount);

            Rect roi = new Rect();
            Mat image = new Mat();
            wait(18000);
            image = api.getMatNavCam();

            Mat ids = new Mat();
            List<Mat> corners = new ArrayList<>();

            try{
                LogT(TAG, "reading ar");
                Aruco.detectMarkers(image, dict, corners, ids);
                LogT(TAG, "ids " + ids.dump());
                for (int i = 0; i < corners.size(); i++) {
                    LogT(TAG, "corners[" + i + "]=" + corners.get(i).dump());
                }

                double[] arCenter = getARCenter(corners);
                double distancePerPixel = getDistancePerPixel(corners);
                Mat arCenterMat = new Mat(1,1, CvType.CV_32FC2);
                arCenterMat.put(0,0, arCenter);

                Mat undistortedPoints = undistortPoints(arCenterMat);
                arCenter[0] = undistortedPoints.get(0,0)[0];
                arCenter[1] = undistortedPoints.get(0,0)[1];

                int[] pixelOffset = getPixelOffsetFromCenter(arCenter,30);
                double[] angleOffset = pixelOffsetToAngleOffset(pixelOffset,angleThreshold);

                result[0] = angleOffset[0];
                result[1] = angleOffset[1];
                result[2] = angleOffset[2];
                result[3] = pixelOffset[0];
                result[4] = pixelOffset[1];
                result[5] = distancePerPixel;

                arContent = (int) ids.get(0, 0)[0];

                LogT(TAG,"ar read successfully");
                return result;
            }
            catch (Exception e){
                LogT(TAG,"an error occurred while reading ar");
            }
            LogT(TAG, "failed to read ar");
            long stop_time = SystemClock.elapsedRealtime();

            loopCount++;
        }
        return result;
    }

    //Misc
    private static long getTime(){ return SystemClock.elapsedRealtime(); }
    private static long getElapsedTime(){ return getTime()-startTime; }
    private static String getElapsedTimeS(){ return " _Time_: " + String.valueOf(getElapsedTime()); }
    private static void LogT(String TAG, String msg){ Log.d(TAG,msg + " " + getElapsedTimeS()); }

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



}

