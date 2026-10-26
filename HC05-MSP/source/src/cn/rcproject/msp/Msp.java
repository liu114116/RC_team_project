package cn.rcproject.msp;

import java.io.ByteArrayOutputStream;
import java.util.Arrays;

/** MSP v1 framing. No Android dependencies: tested on the desktop JVM. */
public final class Msp {
    public interface Listener { void packet(int command, byte[] payload, boolean error); }
    public static byte[] frame(int command, byte[] payload) {
        if (payload.length > 255) throw new IllegalArgumentException("Payload too long");
        ByteArrayOutputStream b = new ByteArrayOutputStream();
        b.write('$'); b.write('M'); b.write('<'); b.write(payload.length); b.write(command);
        int checksum = payload.length ^ command;
        for (byte v : payload) { b.write(v & 255); checksum ^= v & 255; }
        b.write(checksum); return b.toByteArray();
    }
    public static int u16(byte[] b, int i) { return (b[i]&255) | ((b[i+1]&255)<<8); }
    public static int s16(byte[] b, int i) { return (short)u16(b,i); }
    public static boolean validMap(byte[] map) {
        if (map.length != 8) return false;
        boolean[] seen = new boolean[8];
        for (byte b : map) { int i=b&255; if(i>=8 || seen[i]) return false; seen[i]=true; }
        return true;
    }
    public static byte[] benchChannels(byte[] map, int roll, int pitch) {
        return benchChannels(map,roll,pitch,1500);
    }
    public static byte[] benchChannels(byte[] map, int roll, int pitch, int yaw) {
        if (!validMap(map)) throw new IllegalArgumentException("Invalid RX map");
        int[] logical = {Math.max(1400,Math.min(1600,roll)), Math.max(1400,Math.min(1600,pitch)),Math.max(1400,Math.min(1600,yaw)),1000,1000,1000,1000,1000};
        byte[] b = new byte[16];
        for(int i=0;i<8;i++) { int j=map[i]*2, v=logical[i]; b[j]=(byte)v; b[j+1]=(byte)(v>>8); }
        return b;
    }
    public static final class Parser {
        private int state, length, command, pos, checksum;
        private boolean error;
        private byte[] payload;
        private final Listener listener;
        public Parser(Listener listener) { this.listener=listener; }
        public void accept(int v) {
            v &= 255;
            switch(state) {
                case 0: if(v=='$') state=1; break;
                case 1: state=v=='M'?2:(v=='$'?1:0); break;
                case 2: if(v=='>'||v=='!') {error=v=='!';state=3;} else state=v=='$'?1:0; break;
                case 3: length=v;payload=new byte[length];checksum=v;pos=0;state=4;break;
                case 4: command=v;checksum^=v;state=length==0?6:5;break;
                case 5: payload[pos++]=(byte)v;checksum^=v;if(pos==length)state=6;break;
                case 6: state=0;if(v==checksum)listener.packet(command,Arrays.copyOf(payload,length),error);break;
                default: state=0;
            }
        }
    }
}
