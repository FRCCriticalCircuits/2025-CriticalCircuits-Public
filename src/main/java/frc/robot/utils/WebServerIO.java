package frc.robot.utils;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import edu.wpi.first.wpilibj2.command.Commands;

public class WebServerIO extends WebSocketServer {
	public WebServerIO(InetSocketAddress address) {
		super(address);
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

	}

	@Override
	public void onMessage(WebSocket conn, ByteBuffer message) {

	}

	@Override
	public void onError(WebSocket conn, Exception ex) {
		Commands.print("[Server] Error occured on : connection" + conn.getRemoteSocketAddress() + " [" + ex + "]");
	}
	
	@Override
	public void onStart() {
		Commands.print("[Server] Started");
	}
}