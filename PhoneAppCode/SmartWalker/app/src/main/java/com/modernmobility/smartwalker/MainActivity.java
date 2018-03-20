package com.modernmobility.smartwalker;

//REFERENCE: https://developer.android.com/guide/topics/connectivity/bluetooth.html

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.os.Handler;
import android.os.Message;
import android.os.ParcelUuid;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.Math;
import java.nio.ByteBuffer;
import java.util.Set;

    //=================================================================================

/**READ ME
* Currently app acts only as a Bluetooth client
* CAN establish a connection with a Bluetooth server
* CAN send data to server once Bluetooth connected
* CANNOT enable devices' Bluetooth
* CANNOT scan for other Bluetooth devices
* CANNOT pair with other Bluetooth devices*/

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";

    private OutputStream outputStream;
    private InputStream inStream; //May need later

    //Widgets
    TextView txtCmd;
    Button btnConnect;
    Button btnCmd0;
    Button btnCmd1;
    Button btnCmd2;
    Button btnCmd3;
    Button btnCmd4;
    Button btnDisconnect;

    //first paired device
    int position = 0;

    byte[] PhoneData = new byte[1];

    //Test sending floats
    /**
    Button btnTest0;
    Button btnTest1;
    Button btnTest2;
    Button btnTest3;
    Button btnTest4;

    double TestX = 0;
    double TestY = 0;
    double TestO = 0;
    double TestX1 = 1;
    double TestY1 = 0;
    double TestO1 = 0;
    double TestX2 = 0;
    double TestY2 = 1;
    double TestO2 = Math.PI/2;
    double TestX3 = 0;
    double TestY3 = -1;
    double TestO3 = -Math.PI/2;
    double TestX4 = -1;
    double TestY4 = 0;
    double TestO4 = 0;
     */

    TextView tvX;
    TextView tvY;
    TextView tvθ;
    EditText etX;
    EditText etY;
    EditText etθ;
    Button btnSendPose;

    float floatValue;
    byte [] x;
    byte [] y;
    byte [] θ;
    byte [] pose;
    byte [] bytes4 = new byte[4];
    byte [] bytes12 = new byte[12];
    ByteBuffer buffer;


    //=================================================================================

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //Log.d(TAG,"onCreate: created");
        btnConnect = findViewById(R.id.btnConnect);
        btnCmd0 = findViewById(R.id.btnCmd0);
        btnCmd1 = findViewById(R.id.btnCmd1);
        btnCmd2 = findViewById(R.id.btnCmd2);
        btnCmd3 = findViewById(R.id.btnCmd3);
        btnCmd4 = findViewById(R.id.btnCmd4);
        btnDisconnect = findViewById(R.id.btnDisconnect);

        final BluetoothAdapter blueAdapter = BluetoothAdapter.getDefaultAdapter();

        //============================================================================

        /**

        btnTest0 = findViewById(R.id.btnTest0);   //Test sending floats
        btnTest1 = findViewById(R.id.btnTest1);   //Test sending floats
        btnTest2 = findViewById(R.id.btnTest2);   //Test sending floats
        btnTest3 = findViewById(R.id.btnTest3);   //Test sending floats
        btnTest4 = findViewById(R.id.btnTest4);   //Test sending floats

         */

        tvX = findViewById(R.id.tvX);
        tvY = findViewById(R.id.tvY);
        tvθ = findViewById(R.id.tvθ);
        etX = findViewById(R.id.etX);
        etY = findViewById(R.id.etY);
        etθ = findViewById(R.id.etθ);
        btnSendPose = findViewById(R.id.btnSendPose);

        //============================================================================

        /**btnConnect clicked!*/
        btnConnect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnConnect clicked");

                Runnable r = new Runnable() {
                    @Override
                    public void run() {
                        try {
                           // Log.d(TAG,"onCreate: onClick: Run: Run called.");
                            init(blueAdapter);
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        BTConnectTxt.sendEmptyMessage(0);
                    }
                };

                /**Create new thread (Runnable r)*/
                Thread BTConnectionThread = new Thread(r);
                //Log.d(TAG,"onCreate: onClick: BTConnectionThread started.");
                /**Start thread*/
                BTConnectionThread.start();
            }
        });

        /** btnToMe clicked!*/
        btnCmd0.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnToMe clicked");
                PhoneData[0] = 0 ;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");

                    write(PhoneData);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnPark clicked!*/
        btnCmd1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnPark clicked");
                PhoneData[0] = 1;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    PhoneData[0] = 1;
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnStop clicked!*/
        btnCmd2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnStop clicked");
                PhoneData[0] = 2;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnResume clicked!*/
        btnCmd3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnResume clicked");
                PhoneData[0] = 3;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnCancel clicked!*/
        btnCmd4.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnCancel clicked");
                PhoneData[0] = 4;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnDisconnect clicked!*/
        btnDisconnect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                PhoneData[0] = 9;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        //================TESTS=============================================================

        /**HARDCODED TESTS

        //btnTest clicked!
        btnTest0.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                try {
                    Log.d(TAG,"onCreate:  onClick: TEST BUTTON CLICKED!");
                    PhoneData[0] = 10;
                    x = ToByteArray(TestX);
                    y = ToByteArray(TestY);
                    o = ToByteArray(TestO);
                    pose = Pose(x, y, o);
                    write(pose);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        //btnTest1 clicked!
        btnTest1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                try {
                    Log.d(TAG,"onCreate:  onClick: TEST BUTTON CLICKED!");
                    PhoneData[0] = 10;
                    x = ToByteArray(TestX1);
                    y = ToByteArray(TestY1);
                    o = ToByteArray(TestO1);
                    pose = Pose(x, y, o);
                    write(pose);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        //btnTest2 clicked!
        btnTest2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                try {
                    Log.d(TAG,"onCreate:  onClick: TEST BUTTON CLICKED!");
                    PhoneData[0] = 10;
                    x = ToByteArray(TestX2);
                    y = ToByteArray(TestY2);
                    o = ToByteArray(TestO2);
                    pose = Pose(x, y, o);
                    write(pose);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        //btnTest3 clicked!
        btnTest3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                try {
                    Log.d(TAG,"onCreate:  onClick: TEST BUTTON CLICKED!");
                    PhoneData[0] = 10;
                    x = ToByteArray(TestX3);
                    y = ToByteArray(TestY3);
                    o = ToByteArray(TestO3);
                    pose = Pose(x, y, o);
                    write(pose);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        //btnTest4 clicked!
        btnTest4.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                try {
                    Log.d(TAG,"onCreate:  onClick: TEST BUTTON CLICKED!");
                    PhoneData[0] = 10;
                    x = ToByteArray(TestX4);
                    y = ToByteArray(TestY4);
                    o = ToByteArray(TestO4);
                    pose = Pose(x, y, o);
                    write(pose);
                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
         */

        /**VARIABLE TESTS*/

        //btnSendPose clicked!
        btnSendPose.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnDisconnect clicked");
                try {
                    Log.d(TAG,"onCreate:  onClick: SEND POSE BUTTON CLICKED!");
                    PhoneData[0] = 10;
                    x = ToByteArray(Double.parseDouble(etX.getText().toString()));  //Parsing text from editText for X value and passing into byte Array
                    y = ToByteArray(Double.parseDouble(etY.getText().toString()));  //Parsing text from editText for X value and passing into byte Array
                    θ = ToByteArray(Double.parseDouble(etθ.getText().toString()));  //Parsing text from editText for X value and passing into byte Array
                    pose = Pose(x, y, θ);
                    Toasting(pose.toString());
                    write(pose);

                    UpdateCommandTxt(PhoneData[0]);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    //=================================================================================
    //REFERENCE: https://stackoverflow.com/questions/22899475/android-sample-bluetooth-code-to-send-a-simple-string-via-bluetooth

    //This is running on its own thread to avoid slowing down the Main Activity (called from run in the btnConnect onClickListener)

    private void init(BluetoothAdapter blueAdapter) throws IOException {
        Log.d(TAG,"init: called");
        if (blueAdapter != null) {      /**Bluetooth Adapter exists*/
           Log.d(TAG,"init: BT adapter not null");
            if (blueAdapter.isEnabled()) {      /**Bluetooth enabled*/
                Log.d(TAG,"init: BT adapter is enabled");
                Set<BluetoothDevice> bondedDevices = blueAdapter.getBondedDevices();    /**Gets all currently paired devices*/

                if(bondedDevices.size() > 0) {      /**More than zerodevices paired*/
                    Log.d(TAG,"init: Bonded devices greater than zero");
                    Object[] devices = (Object []) bondedDevices.toArray();
                    BluetoothDevice device = (BluetoothDevice) devices[position];
                    ParcelUuid[] uuids = device.getUuids(); /**Gets the UUID of device to establish a connection*/
                    BluetoothSocket socket = device.createRfcommSocketToServiceRecord(uuids[position].getUuid());  /**Create socket using correct UUID*/
                    socket.connect();
                    outputStream = socket.getOutputStream();
                    inStream = socket.getInputStream();
                }
                //Log.e("error", "No appropriate paired devices.");
            } else {
                Log.e("error", "Bluetooth is disabled.");
            }
        }
    }

    //=================================================================================

    Handler BTConnectTxt = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            txtCmd = findViewById(R.id.txtCmd);
            txtCmd.setText(R.string.String_BT_Connected);

        }
    };

    //=================================================================================

    public void write(byte[] PhoneData) throws IOException {
        //Log.d(TAG,"Write: Message sent");
        outputStream.write(PhoneData);  /**Writes to server*/
    }

    //=================================================================================
    public void UpdateCommandTxt(byte PhoneData) {
        switch (PhoneData){
            case 0: //btnToMe clicked!
                txtCmd.setText(R.string.String_ToMe);
                Log.d(TAG,"UpdateCommandTxt: ToMe");
            case 1: //btnPark clicked!
                txtCmd.setText(R.string.String_Park);
                Log.d(TAG,"UpdateCommandTxt: Park");
            case 2: //btnStop clicked!
                txtCmd.setText(R.string.String_Stop);
                Log.d(TAG,"UpdateCommandTxt: Stop");
            case 3: //btnResume clicked!
                txtCmd.setText(R.string.String_Resume);
                Log.d(TAG,"UpdateCommandTxt: Resume");
            case 4: //btnCancel clicked!
                txtCmd.setText(R.string.String_Cancel);
                Log.d(TAG,"UpdateCommandTxt: Cancel");
            case 10: //btnTest clicked!
                txtCmd.setText(R.string.String_Pose);
                Log.d(TAG,"UpdateCommandTxt: Pose");
        }
    }

    //=================================================================================

    public byte[] ToByteArray(double value){
        //Log.d(TAG,"ToByteArray");
        floatValue = (float)value;
        buffer = ByteBuffer.allocate(bytes4.length);
        buffer.putFloat(floatValue);
        return buffer.array();
    }

    //=================================================================================

    public byte[] Pose (byte[] x, byte[] y, byte[] o){
        //Log.d(TAG,"Pose");
        buffer = ByteBuffer.allocate(bytes12.length);
        buffer.put(x).put(y).put(o);
        return buffer.array();
    }

    //=================================================================================

    public void Toasting(String s){
        Context context = getApplicationContext();
        CharSequence text = "Hello toast!";
        int duration = Toast.LENGTH_SHORT;

        Toast.makeText(context, text, duration).show();
    }

   /* //=================================================================================
   //May need this later

    public void run() {
        final int BUFFER_SIZE = 1024;
        byte[] buffer = new byte[BUFFER_SIZE];
        int bytes = 0;
        int b = BUFFER_SIZE;

        while (true) {
            try {
                bytes = inStream.read(buffer, bytes, BUFFER_SIZE - bytes);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    //================================================================================= */
}