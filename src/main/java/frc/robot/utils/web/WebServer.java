package frc.robot.utils.web;

import java.net.InetSocketAddress;

import org.json.JSONArray;
import org.json.JSONObject;

import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.SolverRequest;
import frc.robot.utils.structures.SolverResult;

public class WebServer {
    private static WebServer instance;
    private WebServerIO wsServer;

    private WebServer(){
        wsServer = new WebServerIO(new InetSocketAddress("10.90.62.2", 9062));
        if(DriveStationIO.isTest()){
            new Thread(
            () -> {
                wsServer.run();
            }
            ).start();
        }
    }

    public static WebServer getInstance(){
        if(instance == null) instance = new WebServer();
        return instance;
    }

    public void sendSolverRequest(SolverRequest request){
        JSONObject object = new JSONObject();
        JSONArray jsonArray = new JSONArray(request.asArray());
        int requestId = request.getId();

        object.put("DatapackType", 1);
        object.put("request", jsonArray);
        object.put("requestId", requestId);

        wsServer.expectedResultId(requestId);
        wsServer.broadcast(object.toString());
    }

    public boolean hasSolverResult(){
        return wsServer.hasSolverResult();
    }

    public SolverResult getSolverResult(){
        return wsServer.getSolverResult();
    }

    public synchronized AutoAimSetting getAutoAimSettings(){
        return wsServer.getSettings();
    }

    /**
     * Update the settings when control through operator joystick
     * @param settings target settings
     */
    public void updateSetting(AutoAimSetting settings){
        wsServer.updateSetting(settings);
    }
}
