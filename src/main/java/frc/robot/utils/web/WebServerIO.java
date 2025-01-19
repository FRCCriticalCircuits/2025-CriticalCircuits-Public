package frc.robot.utils.web;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.SolverResult;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;

public class WebServerIO extends WebSocketServer {
	private AutoAimSetting settings = Constants.DEFAULT_SETTING;
	private SolverResult solverResult;
	private int resultId = 0, expectedResultId = 0;

	public WebServerIO(InetSocketAddress address) {
		super(address);
	}

	public AutoAimSetting getSettings(){
		return this.settings;
	}

	public SolverResult getSolverResult(){
		return solverResult;
	}

	public void expectedResultId(int id){
		this.expectedResultId = id;
	}

	public boolean hasSolverResult(){
		return expectedResultId == resultId;
	}

	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		Commands.print("[Server] Connection From: " + conn.getRemoteSocketAddress());
	}

	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
		Commands.print("[Server] " + conn.getRemoteSocketAddress() + "disconnected with exit code " + code + " [" + reason + "]");
	}

	@Override
	public void onMessage(WebSocket conn, String message) {
		JSONObject object = new JSONObject(message);

		int requestId = object.getInt("DatapackType");

		/* 
		 * ID 0: Update AutoAim Settings Request from Driver 
		 * ID 1: Solver Request - not gonna receive it
		 * ID 2: Solver Result
		 */
		switch(requestId){
			case 0: 
				Spot spot = Spot.valueOf(object.getJSONObject("settings").getInt("spot"));
				Level level = Level.valueOf(object.getJSONObject("settings").getInt("level"));
				Mode mode = Mode.valueOf(object.getJSONObject("settings").getInt("mode"));
				this.settings = new AutoAimSetting(spot, level, mode);
				break;
			case 1:
				break;
			default:
				JSONArray jsonArray = object.getJSONArray("result");
				double[][] dataArray = new double[jsonArray.length()][2];
				for (int i = 0; i < jsonArray.length(); i++) {
					dataArray[i] = new double[2];
					JSONArray innerArray = jsonArray.getJSONArray(i);
					for(int j = 0; j < 2; j++){
						dataArray[i][j] = innerArray.getDouble(j);
					}
				}
				solverResult = new SolverResult(dataArray);
				resultId = object.getInt("resultId");
		}
	}

	@Override
	public void onMessage(WebSocket conn, ByteBuffer message) {}

	@Override
	public void onError(WebSocket conn, Exception ex) {
		Commands.print("[Server] Error occured on : connection" + conn.getRemoteSocketAddress() + " [" + ex + "]");
	}
	
	@Override
	public void onStart() {
		Commands.print("[Server] Started");
	}
}