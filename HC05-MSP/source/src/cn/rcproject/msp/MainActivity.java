package cn.rcproject.msp;
import android.Manifest;
import android.app.*;
import android.bluetooth.*;
import android.content.pm.PackageManager;
import android.os.*;
import android.webkit.*;
import android.view.*;
import java.io.*;
import java.util.*;
import java.util.concurrent.*;
import org.json.*;

/** Telemetry and opt-in motor-disconnected RC bench testing. */
public class MainActivity extends Activity {
 private final Handler ui=new Handler(Looper.getMainLooper());
 private final ExecutorService worker=Executors.newSingleThreadExecutor();
 private final Object writeLock=new Object();
 private volatile BluetoothSocket socket;
 private volatile boolean connected,connecting,active;
 private volatile int generation;
 private volatile boolean testing;
 private volatile int rollInput=1500,pitchInput=1500,yawInput=1500;
 private volatile long lastTouch;
 private final ScheduledExecutorService rcWorker=Executors.newSingleThreadScheduledExecutor();
 private volatile byte[] rxMap;
 private volatile long lastReply;
 private WebView web;
 private final JSONObject telemetry=new JSONObject();
 private final StringBuilder logs=new StringBuilder();
 private int tick;
 @Override public void onCreate(Bundle b){super.onCreate(b);
  getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
  getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,WindowManager.LayoutParams.FLAG_FULLSCREEN);
  getWindow().setStatusBarColor(0xfff6f8fa);getWindow().setNavigationBarColor(0xfff6f8fa);
  getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN|View.SYSTEM_UI_FLAG_HIDE_NAVIGATION|View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY|View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN|View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION|View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
  web=new WebView(this);web.setBackgroundColor(0xfff6f8fa);setContentView(web);
  web.getSettings().setJavaScriptEnabled(true);
  web.getSettings().setAllowFileAccess(false);web.getSettings().setAllowContentAccess(false);
  web.getSettings().setAllowFileAccessFromFileURLs(false);web.getSettings().setAllowUniversalAccessFromFileURLs(false);
  web.addJavascriptInterface(new Bridge(),"Native");
  web.setWebViewClient(new WebViewClient(){@Override public boolean shouldOverrideUrlLoading(WebView v,WebResourceRequest r){return true;}});
  web.loadUrl("file:///android_asset/index.html");
  rcWorker.scheduleAtFixedRate(()->{
   if(!testing||!active||!connected)return;
   if(SystemClock.elapsedRealtime()-lastReply>1500){stopTesting();return;}
   if(SystemClock.elapsedRealtime()-lastTouch>350){rollInput=pitchInput=yawInput=1500;}
   synchronized(writeLock){if(!testing||!active||!connected)return;try{send(200,Msp.benchChannels(rxMap,rollInput,pitchInput,yawInput));}catch(Exception e){stopTesting();addLog("通道发送失败："+e.getMessage());}}
  },0,50,TimeUnit.MILLISECONDS);
 }
 private void stopTesting(){testing=false;rollInput=pitchInput=yawInput=1500;}
 public final class Bridge {
  @JavascriptInterface public void testing(boolean enable){ui.post(()->{
   if(!enable){stopTesting();publish();return;}
   if(!active||!connected||rxMap==null||SystemClock.elapsedRealtime()-lastReply>1500){addLog("等待飞控应答和有效通道映射");return;}
   new AlertDialog.Builder(MainActivity.this).setTitle("无电机通道测试").setMessage("确认电机未连接、全部螺旋桨已拆除，并且 ARM/ANGLE/BARO 按约定由 AUX 高值激活。测试油门固定 1000、AUX1–4 固定 1000，不开放解锁或升降。")
    .setNegativeButton("取消",null).setPositiveButton("确认并开始",(d,w)->{if(active&&connected&&rxMap!=null&&SystemClock.elapsedRealtime()-lastReply<1500){rollInput=pitchInput=yawInput=1500;lastTouch=SystemClock.elapsedRealtime();testing=true;publish();}}).show();
  });}
  @JavascriptInterface public void sticks(int roll,int pitch,int yaw){if(!testing||!active)return;rollInput=Math.max(1400,Math.min(1600,roll));pitchInput=Math.max(1400,Math.min(1600,pitch));yawInput=Math.max(1400,Math.min(1600,yaw));lastTouch=SystemClock.elapsedRealtime();}
  @JavascriptInterface public void page(boolean control){if(!control)stopTesting();}

  @JavascriptInterface public void connect(){ui.post(()->choose());}
  @JavascriptInterface public void disconnect(){ui.post(()->MainActivity.this.disconnect("手动断开"));}
 }
 private void renderEvent(String event){if(web!=null)web.evaluateJavascript("window.nativeEvent&&window.nativeEvent("+JSONObject.quote(event)+")",null);}
 private void put(String k,Object v){synchronized(telemetry){try{telemetry.put(k,v);}catch(JSONException ignored){}}}
 private void publish(){String data;synchronized(telemetry){try{telemetry.put("testing",testing);telemetry.put("connected",connected);telemetry.put("fresh",connected&&lastReply>0&&SystemClock.elapsedRealtime()-lastReply<1500);telemetry.put("log",logs.toString());}catch(JSONException ignored){}data=telemetry.toString();}if(web!=null)web.evaluateJavascript("window.nativeData&&window.nativeData("+data+")",null);}
 private void addLog(String s){ui.post(()->{logs.append(s).append('\n');if(logs.length()>3000)logs.delete(0,logs.length()-2500);publish();});}
 private final Runnable loop=new Runnable(){public void run(){
  if(active&&connected){final int token=generation;final int cmd=new int[]{108,101,105,109,110}[tick++%5];
   worker.execute(()->{if(token!=generation||!connected)return;try{send(cmd,new byte[0]);}catch(Exception e){if(token==generation)disconnect("通信失败："+e.getMessage());}});
  }
  publish();if(active)ui.postDelayed(this,200);
 }};
    private void choose(){
        if(Build.VERSION.SDK_INT>=31 && checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT)!=PackageManager.PERMISSION_GRANTED){requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT},7);return;}
        BluetoothManager m=(BluetoothManager)getSystemService(BLUETOOTH_SERVICE);BluetoothAdapter adapter=m.getAdapter();
        if(adapter==null||!adapter.isEnabled()){addLog("先在系统设置打开蓝牙，并与 HC-05 配对");return;}
        List<BluetoothDevice> devices=new ArrayList<>(adapter.getBondedDevices());
        if(devices.isEmpty()){addLog("没有已配对设备，请先在系统蓝牙设置配对 HC-05");return;}
        String[] names=new String[devices.size()];for(int i=0;i<names.length;i++)names[i]=devices.get(i).getName()+"\n"+devices.get(i).getAddress();
        new AlertDialog.Builder(this).setTitle("选择 HC-05").setItems(names,(d,n)->connect(devices.get(n))).show();
    }
    private void connect(BluetoothDevice device){
        if(connecting){addLog("正在连接，请稍候");return;}
        disconnect("准备连接");connecting=true;final int token=generation;
        worker.execute(()->{BluetoothSocket s=null;try{
            s=device.createRfcommSocketToServiceRecord(UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"));
            if(token!=generation)return;socket=s;
            final BluetoothSocket pending=s;
            ui.postDelayed(()->{if(token==generation&&connecting){disconnect("蓝牙连接超时");try{pending.close();}catch(Exception ignored){}}},12000);
            s.connect();
            if(!active||token!=generation){s.close();if(token==generation)disconnect("连接过程中进入后台，请重新连接");return;}
            connected=true;connecting=false;lastReply=0;ui.post(()->renderEvent("connected"));addLog("蓝牙 SPP 已连接，正在查询飞控");
            final BluetoothSocket current=s;
            new Thread(()->read(current,token),"msp-reader").start();
            send(1,new byte[0]);send(2,new byte[0]);send(3,new byte[0]);send(4,new byte[0]);send(64,new byte[0]);send(101,new byte[0]);send(119,new byte[0]);
        }catch(Exception e){if(token==generation)disconnect("连接失败："+e.getMessage());}
        finally{if(token!=generation&&s!=null)try{s.close();}catch(Exception ignored){}}
        });
    }
    private void send(int cmd,byte[] payload)throws IOException{ synchronized(writeLock){BluetoothSocket s=socket;if(!connected||s==null)throw new IOException("未连接");s.getOutputStream().write(Msp.frame(cmd,payload));}}
    private void read(BluetoothSocket s,int token){Msp.Parser parser=new Msp.Parser((c,b,e)->{if(token==generation)packet(c,b,e);});try{InputStream in=s.getInputStream();int v;while(token==generation&&(v=in.read())!=-1)parser.accept(v);if(token==generation)disconnect("蓝牙连接已关闭");}catch(Exception e){if(token==generation)disconnect("连接中断："+e.getMessage());}}

 private void packet(int c,byte[] b,boolean error){
  if(error){addLog("飞控拒绝命令 "+c);return;}lastReply=SystemClock.elapsedRealtime();
  if(c==64){if(Msp.validMap(b)){rxMap=b;put("mapped",true);}else addLog("通道映射无效");}
  if(c==1&&b.length>=3)put("api",(b[1]&255)+"."+(b[2]&255));
  if(c==2)put("firmware",new String(b,java.nio.charset.StandardCharsets.US_ASCII));
  if(c==4&&b.length>=4)put("board",new String(b,0,4,java.nio.charset.StandardCharsets.US_ASCII));
  if(c==3&&b.length>=3)put("version",(b[0]&255)+"."+(b[1]&255)+"."+(b[2]&255));
  if(c==101&&b.length>=6){int mask=Msp.u16(b,4);put("acc",(mask&1)!=0);put("baro",(mask&2)!=0);}
  if(c==119)put("modeNames",new String(b,java.nio.charset.StandardCharsets.US_ASCII));
  if(c==108&&b.length>=6){put("roll",Msp.s16(b,0)/10.0);put("pitch",Msp.s16(b,2)/10.0);put("yaw",Msp.s16(b,4));}
  if(c==109&&b.length>=6){int cm=(b[0]&255)|((b[1]&255)<<8)|((b[2]&255)<<16)|((b[3]&255)<<24);put("altitude",cm/100.0);}
  if(c==110&&b.length>=1)put("voltage",(b[0]&255)/10.0);
  if(c==105){JSONArray a=new JSONArray();for(int i=0;i<Math.min(8,b.length/2);i++)a.put(Msp.u16(b,i*2));put("channels",a);}
 }
 private void disconnect(String reason){stopTesting();generation++;connected=false;connecting=false;rxMap=null;lastReply=0;BluetoothSocket s=socket;socket=null;if(s!=null)try{s.close();}catch(Exception ignored){}synchronized(telemetry){Iterator<String> keys=telemetry.keys();while(keys.hasNext()){keys.next();keys.remove();}}ui.post(()->{renderEvent("disconnected");addLog(reason);});}
 @Override public void onRequestPermissionsResult(int request,String[] permissions,int[] results){super.onRequestPermissionsResult(request,permissions,results);if(request==7&&results.length>0&&results[0]==PackageManager.PERMISSION_GRANTED)choose();else addLog("请允许附近设备权限后再连接");}
 @Override public void onResume(){super.onResume();active=true;ui.removeCallbacks(loop);ui.post(loop);}
 @Override public void onPause(){active=false;ui.removeCallbacks(loop);stopTesting();super.onPause();}
 @Override public void onDestroy(){disconnect("应用关闭");rcWorker.shutdownNow();worker.shutdownNow();web.removeJavascriptInterface("Native");web.destroy();web=null;super.onDestroy();}
}
