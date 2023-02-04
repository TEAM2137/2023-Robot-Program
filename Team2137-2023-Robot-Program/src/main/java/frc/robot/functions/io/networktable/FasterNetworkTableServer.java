package frc.robot.functions.io.networktable;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.net.*;
import java.net.http.HttpClient;
import java.net.http.HttpResponse;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class FasterNetworkTableServer {
    public final FasterNetworkTable rootTable;

    private DatagramSocket udpSocket;
    private DatagramSocket tcpSocket;
    private InetAddress clientAddress;
    private final ByteBuffer buffer = ByteBuffer.allocate(256);

    private static final int fastSendRateMS = 50;
    private static final int slowSendRateMS = 2_000;
    private ScheduledThreadPoolExecutor threadedExecutor;
    private static int jsonChangeToFlush = 0;

    public static void main(String... args) {
        FasterNetworkTableServer server = new FasterNetworkTableServer("localhost");
        FasterNetworkTableValue<Double> testVal = new FasterNetworkTableValue<Double>("Test", 0.0);
        testVal.setValue(0.0);
        server.rootTable.add(testVal);

        double pos = 0.0;
        while(true) {
            pos++;
            FasterNetworkTable table = new FasterNetworkTable("Table" + pos);
            server.rootTable.add(table);
            testVal.setValue(pos);
            server.addValueToUDPPacket(testVal);
            //server.postJsonStructure();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public FasterNetworkTableServer(String ipAddress) {
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        rootTable = new FasterNetworkTable("Root");
        rootTable.setPath("/");

        try {
//            clientAddress = InetAddress.getByName(ipAddress);
            clientAddress = InetAddress.getLocalHost();
            udpSocket = new DatagramSocket();
            tcpSocket = new DatagramSocket();

            threadedExecutor = new ScheduledThreadPoolExecutor(1);
            threadedExecutor.scheduleAtFixedRate(this::sendDataPacket, 0, fastSendRateMS, TimeUnit.MILLISECONDS);
            threadedExecutor.scheduleAtFixedRate(this::postJsonStructure, 0, slowSendRateMS, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void sendDataPacket() {
        synchronized (buffer) {
            if(buffer.position() < 6)
                return;

            byte[] array = buffer.array();
            DatagramPacket packet = new DatagramPacket(array, array.length, clientAddress, 57);
            System.out.println(bytesToHex(array));

            try {
                udpSocket.send(packet);
                System.out.println("Sent UDP Packet");
                buffer.clear();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void postJsonStructure() {
        if(jsonChangeToFlush == 0) return;

        try {
            StringBuilder builder = new StringBuilder();
            builder.append("[\n");
            rootTable.convertToJson(builder);
            builder.append("]\n");

            System.out.println(builder.toString());

            byte[] out = builder.toString().getBytes(StandardCharsets.UTF_8);
            DatagramPacket packet = new DatagramPacket(out, out.length, clientAddress, 58);

            tcpSocket.connect(clientAddress, 58);
            tcpSocket.send(packet);
            System.out.println("Sent");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = HEX_ARRAY[v >>> 4];
            hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
        }
        return new String(hexChars);
    }

    protected synchronized void addValueToUDPPacket(FasterNetworkTableValue<?> value) {
        if(buffer.remaining() < 12) {
            sendDataPacket();
        }

        synchronized (buffer) {
            buffer.put((byte) 0xA5);
            buffer.putInt(value.getID());

            switch (value.getType()) {
//            case String:
//                ByteBuffer buffer = ByteBuffer.allocate(4);
//                addByteBuffer(buffer, value.hashCode(), (byte) 0x01);
//                break;
                case Double:
                    buffer.put((byte) 0x02);
                    buffer.putDouble(((FasterNetworkTableValue<Double>) value).getValue());
                    break;
                case Int:
                    buffer.put((byte) 0x03);
                    buffer.putInt(((FasterNetworkTableValue<Integer>) value).getValue());
                    break;
                case Boolean:
                    buffer.put((byte) 0x04);
                    if (((FasterNetworkTableValue<Boolean>) value).getValue())
                        buffer.put((byte) 0x01);
                    else
                        buffer.put((byte) 0x00);
                    break;
            }
        }
    }

    protected static void registerChange() {
        jsonChangeToFlush++;
    }

    public FasterNetworkTable getRootTable() {
        return rootTable;
    }
}
