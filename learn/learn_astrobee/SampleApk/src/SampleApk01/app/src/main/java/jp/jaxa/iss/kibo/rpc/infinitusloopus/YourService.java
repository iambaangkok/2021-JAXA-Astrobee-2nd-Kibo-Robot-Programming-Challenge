package jp.jaxa.iss.kibo.rpc.infinitusloopus;

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
        api.judgeSendStart();

        moveToWrapper(10.6, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
        moveToWrapper(11, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
        moveToWrapper(11, -5.7, 5, 0, 0, -0.7071068, 0.7071068);
        moveToWrapper(11.5, -5.7, 4.5, 0, 0, 0, 1);
        moveToWrapper(11, -6, 5.55, 0, -0.7071068, 0, 0.7071068);

        api.laserControl(true);
        moveToWrapper(11.1, -6, 5.55, 0, -0.7071068, 0, 0.7071068);

        api.judgeSendFinishSimulation();
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

}

