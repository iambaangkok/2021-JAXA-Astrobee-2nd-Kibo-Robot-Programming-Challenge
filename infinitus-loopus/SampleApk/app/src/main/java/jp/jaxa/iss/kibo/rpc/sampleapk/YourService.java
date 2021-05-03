package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

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

        api.startMission();
        // move(11.71, -9.53, 5.35, 0, 0, 0, 1);

        move(11.21,-9.8,4.79,0,0,-0.707,0.707);
        Log.i("Check" , "Finished - move");

        String contents = readQR();
        //api.sendDiscoveredQR(contents);
        Log.i("Check" , "Finished - .readQR");
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

    public void move(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);

        api.moveTo(point, quaternion, true);

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

    public String readQR() {
        String contents = null;

        Log.i("QRCode" , "GetMat");
        Mat notPic = api.getMatNavCam();
        Log.i("QRCode" , "Finished - GetMat");

        Log.i("QRCode" , "Resize");
        double scale = 0.5;
        Mat pic = new Mat();
        Size newPicSize = new Size(scale * notPic.size().width , scale * notPic.size().height);
        Imgproc.resize(notPic , pic , newPicSize);
        Log.i("QRCode" , "Finished - Resize");

        Log.i("QRCode" , "Convert To Bitmap");
        Bitmap bMap = Bitmap.createBitmap(pic.width(), pic.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(pic, bMap);
        Log.i("QRCode" , "Finished - Convert To Bitmap");

        Log.i("QRCode" , "Convert To BinaryBitmap");
        int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];
        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
        Log.i("QRCode" , "Finished - Convert To BinaryBitmap");

        Log.i("QRCode" , "Decoding");
        Reader reader = new MultiFormatReader();
        try {
            com.google.zxing.Result result = reader.decode(bitmap);
            contents = result.getText();
        } catch (NotFoundException e) { e.printStackTrace(); }
        catch (ChecksumException e) { e.printStackTrace(); }
        catch (FormatException e) { e.printStackTrace(); }
        Log.i("QRCode" , "Finished - Decoding");

        Log.i("QRCode" , "The content is " + contents);
        return contents;
    }
}

