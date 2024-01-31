package frc.robot.util;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;

public class Dashboard extends WebSocketServer {
    private class KeyInfo {
        public boolean pressed;
        public double timestamp;

        public KeyInfo(boolean p, double t) {
            pressed = p;
            timestamp = t;
        }
    }

    private final int UpdateRate = 40; // update in ms

    private static HashMap<String, Supplier<Object>> OutgoingValues = new HashMap<String, Supplier<Object>>();
    private static HashMap<String, KeyInfo> Keys = new HashMap<String, KeyInfo>();
    private static HashMap<String, Consumer<String>> IncomingValues = new HashMap<String, Consumer<String>>();

    /** Used to add values to send to dashboard. */
    public static void push(String name, Supplier<Object> value) { 
        OutgoingValues.put(name, value);
    }

    /** Register Functions to run based off of incoming values */
    public static void register(String name, Consumer<String> value) {
        IncomingValues.put(name, value);
    }

    public static boolean getKey(String name) {
        if (!Keys.containsKey(name) || Timer.getFPGATimestamp() - Keys.get(name).timestamp > 0.5) {
            return false;
        }
        
        return Keys.get(name).pressed;
    }

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

    private void dataReceived(String key, String value) {
        switch (key) {
            default:           
                KeyInfo info = new KeyInfo((value.length() > 2) ? true : false, Timer.getFPGATimestamp()); 
                Keys.put(key, info);

                if (IncomingValues.containsKey(key))
                    IncomingValues.get(key).accept(value);
                break;
        }
    }

    @SuppressWarnings("unchecked")
    private String dataToSend() {
        JSONObject obj = new JSONObject();

        obj.put("selected_auto", "test");
        obj.put("auto", new String[] {"auto 1", "auto 2"});
        obj.put("voltage", RobotController.getBatteryVoltage());
        obj.put("time", Timer.getMatchTime());
        

        JSONObject pose = new JSONObject();
        pose.put("X", Swerve.get().getPose().getX());
        pose.put("Y", Swerve.get().getPose().getY());
        pose.put("R", Swerve.get().getPose().getRotation().getDegrees());
        obj.put("pose", pose);

        obj.put("debug", new JSONArray());

        for (String key : OutgoingValues.keySet()) {
            obj.put(key, OutgoingValues.get(key).get());
        }

        return obj.toJSONString();
    }  

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("Dashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        (new Thread(() -> {
            while (conn.isOpen()) {

                conn.send(dataToSend());

                try {
                    Thread.sleep(UpdateRate);
                } catch (InterruptedException e) {
                    Log.fatalException("Dashboard", "Thread Interrupted", e);
                }
            }
        })).start();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        Keys.clear();
        //Log.info("Dashboard", conn.getRemoteSocketAddress().getHostName() + " has closed its connection.");
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        String[] parts = message.split(":"); // Message Comes in "Key:Value pair"
        dataReceived(parts[0], parts[1]);

        Log.info("Dashboard: " + conn.getRemoteSocketAddress().getHostName(), "Message: " + message);
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        Keys.clear();
        Log.fatalException("Dashboard", "Error", ex);
    }

    @Override
    public void onStart() {
        Log.info("Dashboard", "Server has started on port 5805.");
    }
}
