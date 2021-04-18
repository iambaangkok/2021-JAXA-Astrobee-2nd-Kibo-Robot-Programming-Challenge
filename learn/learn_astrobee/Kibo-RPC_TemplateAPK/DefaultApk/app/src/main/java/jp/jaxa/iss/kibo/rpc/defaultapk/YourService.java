package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // start this run
        api.judgeSendStart();
        // move Astrobee from the starting point to P1 1
        Point point = new Point(1.1, 2.2, 3.3);
        Quaternion quaternion = new Quaternion(4.4, 5.5, 6.6, 7.7);
        api.moveTo(point, quaternion, true);
        // :
        // once Astrobee came to P1 1, get a camera image
        Bitmap snapshot = api.getBitmapNavCam();
        // read the QR code in the image and get the x axis coordinate value of P3
        //String valueX = …;
        // send the result to scoring module api.judgeSendDiscoveredQR(0, valueX);
        // implement some other functions or repeat for P1 2, P1 3, …
        // :
        // once Astrobee came to P3, get a camera image
        Bitmap snapshot = api.getBitmapNavCam();
        // read the AR tag in the image String markerId = …;
        // send the result to the scoring module api.judgeSendDiscoveredAR(markerId);
        // some other functions
        // :
        // turn on the laser light, api.laserControl(true);
        // take snapshots to evaluate the accuracy of laser pointing and finish this run
        api.judgeSendFinishSimulation();
        sendData(MessageType.JSON, "data", "SUCCESS:defaultapk runPlan1");
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

}

