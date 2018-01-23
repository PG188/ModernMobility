package com.modernmobility.smartwalker;

//REFERENCE: https://developer.android.com/guide/topics/connectivity/bluetooth.html

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Handler;
import android.os.Message;
import android.os.ParcelUuid;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
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
    //private InputStream inStream; //May need later

    //Widgets
    TextView txtCmd;
    Button btnConnect;
    Button btnToMe;
    Button btnPark;
    Button btnStop;
    Button btnResume;
    Button btnCancel;
    Button btnDisconnect;

    //first paired device
    int position = 0;

    byte PhoneData;

    //=================================================================================

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //Log.d(TAG,"onCreate: created");
        btnConnect = findViewById(R.id.btnConnect);
        btnToMe = findViewById(R.id.btnToMe);
        btnPark = findViewById(R.id.btnPark);
        btnStop = findViewById(R.id.btnStop);
        btnResume = findViewById(R.id.btnResume);
        btnCancel = findViewById(R.id.btnCancel);
        btnDisconnect = findViewById(R.id.btnDisconnect);

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
                            init();
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
        btnToMe.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnToMe clicked");
                PhoneData = 0;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");

                    write(PhoneData);
                    UpdateCommandTxt(PhoneData);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnPark clicked!*/
        btnPark.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnPark clicked");
                PhoneData = 1;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    PhoneData = 1;
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnStop clicked!*/
        btnStop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnStop clicked");
                PhoneData = 2;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnResume clicked!*/
        btnResume.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnResume clicked");
                PhoneData = 3;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        /**btnCancel clicked!*/
        btnCancel.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //Log.d(TAG,"onCreate: onClick: btnCancel clicked");
                PhoneData = 4;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData);
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
                PhoneData = 9;
                try {
                    //Log.d(TAG,"onCreate: onClick: Message written");
                    write(PhoneData);
                    UpdateCommandTxt(PhoneData);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    //=================================================================================
    //REFERENCE: https://stackoverflow.com/questions/22899475/android-sample-bluetooth-code-to-send-a-simple-string-via-bluetooth

    //This is running on its own thread to avoid slowing down the Main Activity (called from run in the btnConnect onClickListener)

    private void init() throws IOException {
        //Log.d(TAG,"init: called");
        BluetoothAdapter blueAdapter = BluetoothAdapter.getDefaultAdapter();
        if (blueAdapter != null) {      /**Bluetooth Adapter exists*/
           // Log.d(TAG,"init: BT adapter not null");
            if (blueAdapter.isEnabled()) {      /**Bluetooth enabled*/
                //Log.d(TAG,"init: BT adapter is enabled");
                Set<BluetoothDevice> bondedDevices = blueAdapter.getBondedDevices();    /**Gets all currently paired devices*/

                if(bondedDevices.size() > 0) {      /**More than zerodevices paired*/
                    //Log.d(TAG,"init: Bonded devices greater than zero");
                    Object[] devices = (Object []) bondedDevices.toArray();
                    BluetoothDevice device = (BluetoothDevice) devices[position];
                    ParcelUuid[] uuids = device.getUuids(); /**Gets the UUID of device to establish a connection*/
                    BluetoothSocket socket = device.createRfcommSocketToServiceRecord(uuids[0].getUuid());  /**Create socket using correct UUID*/
                    socket.connect();
                    outputStream = socket.getOutputStream();
                    //inStream = socket.getInputStream();
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
            txtCmd.setText("Bluetooth connection successful!");

        }
    };

    //=================================================================================

    public void write(byte PhoneData) throws IOException {
        //Log.d(TAG,"Write: Message sent");
        outputStream.write(PhoneData);   /**Writes to server*/
    }

    //=================================================================================
    public void UpdateCommandTxt(byte PhoneData) {
        switch (PhoneData){
            case 0: //btnToMe clicked!
                txtCmd.setText("SmartWalker is Navigating towards the the user.");
                //Log.d(TAG,"UpdateCommandTxt: ToMe");
            case 1: //btnPark clicked!
                txtCmd.setText("SmartWalker is Parking.");
                //Log.d(TAG,"UpdateCommandTxt: Park");
            case 2: //btnStop clicked!
                txtCmd.setText("SmartWalker is has stopped.");
                //Log.d(TAG,"UpdateCommandTxt: Stop");
            case 3: //btnResume clicked!
                txtCmd.setText("SmartWalker is resuming.");
                //Log.d(TAG,"UpdateCommandTxt: Resume");
            case 4: //btnCancel clicked!
                txtCmd.setText("SmartWalker is Navigating towards the the user.");
                //Log.d(TAG,"UpdateCommandTxt: Cancel");
        }
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