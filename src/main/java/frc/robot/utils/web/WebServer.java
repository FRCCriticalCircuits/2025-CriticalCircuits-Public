package frc.robot.utils.web;

import java.net.InetSocketAddress;

import org.json.JSONArray;
import org.json.JSONObject;

import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.SolverRequest;
import frc.robot.utils.structures.SolverResult;

public class WebServer {
    private static WebServer instance;
    private WebServerIO wsServer;

    private WebServer(){
        wsServer = new WebServerIO(new InetSocketAddress("localhost", 9062));
        wsServer.run();
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

    public AutoAimSetting getAutoAimSettings(){
        return wsServer.getSettings();
    }
}
