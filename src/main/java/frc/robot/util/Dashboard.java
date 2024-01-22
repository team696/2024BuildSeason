package frc.robot.util;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class Dashboard extends WebSocketServer {
    private Dashboard(int port) throws UnknownHostException   {
        super(new InetSocketAddress(port));
    }

    private Dashboard(InetSocketAddress address) {
        super(address);
    }

    public static void startServer() {
        Dashboard s;
        try {
            s = new Dashboard(5805);
            s.setReuseAddr(true);
            s.start();  
        } catch (UnknownHostException e) {
            Log.fatalException("Dashboard", "Failed To Start Server.", e);   
        }
    }

    @SuppressWarnings("unchecked")
    private String dataToSend() {
        JSONObject obj = new JSONObject();
        obj.put("selected_auto", "test");
        obj.put("auto", new String[] {"auto 1", "auto 2"});
        obj.put("voltage", RobotController.getBatteryVoltage());
        obj.put("time", Timer.getMatchTime());

        obj.put("debug", new JSONArray());

        return obj.toJSONString();
    }  

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("Dashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        (new Thread(() -> {
            while (conn.isOpen()) {

                conn.send(dataToSend());

                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    Log.fatalException("Dashboard", "Thread Interrupted", e);
                }
            }
        })).start();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        Log.info("Dashboard", conn.getRemoteSocketAddress().getHostName() + " has closed its connection.");
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        String[] parts = message.split(":"); // Message Comes in "Key:Value pair"
        String value = parts[1];

        switch (parts[0]) { 
            case "selectauto":
                Log.info("Dashboard", "Set Auto to " + value);
                break;
            default: 
                Log.info("Dashboard: " + conn.getRemoteSocketAddress().getHostName(), "Message: " + message);
                break;
        }
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        Log.fatalException("Dashboard", "Error", ex);
    }

    @Override
    public void onStart() {
        Log.info("Dashboard", "Server has started on port 5805.");
    }
}
