import cn.rcproject.msp.Msp;
import java.util.*;
public class MspTest {
 public static void main(String[] args){
  if(!Arrays.equals(Msp.frame(1,new byte[0]),new byte[]{36,77,60,0,1,1}))throw new AssertionError("framing");
  final int[] calls={0}; Msp.Parser p=new Msp.Parser((c,b,e)->{if(c!=108||b.length!=2||b[0]!=7||!e)throw new AssertionError();calls[0]++;});
  byte[] reply={36,77,33,2,108,7,0,105};for(byte b:reply)p.accept(b);
  if(calls[0]!=1)throw new AssertionError("reply");
  reply[7]=0;for(byte b:reply)p.accept(b);if(calls[0]!=1)throw new AssertionError("checksum");
  byte[] map={1,2,3,0,4,5,6,7};byte[] channels=Msp.benchChannels(map,9999,-1);
  if(Msp.u16(channels,0)!=1000||Msp.u16(channels,2)!=1600||Msp.u16(channels,4)!=1400||Msp.u16(channels,6)!=1500)throw new AssertionError("mapping/clamp");
  if(Msp.validMap(new byte[]{0,0,2,3,4,5,6,7}))throw new AssertionError("invalid map");
  System.out.println("PASS: framing, parser, checksum rejection, RX mapping, throttle and clamp");
 }
}
