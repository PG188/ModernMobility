package com.modernmobility.smartwalker;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.ParcelUuid;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;

    //=================================================================================

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";
    private OutputStream outputStream;
    private InputStream inStream;
    int position = 0;
    Button btnSend;
    Button btnConnect;

    //=================================================================================

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Log.d(TAG,"onCreate: created");
        btnConnect = (Button) findViewById(R.id.btnConnect);
        btnSend = (Button) findViewById(R.id.btnSend);

        btnConnect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG,"onCreate: onClick: Connect button clicked");
                try {
                    Log.d(TAG,"onCreate: onClick: init called");
                    init();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        btnSend.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG,"onCreate: onClick: Send button clicked");
                String s = "Hello from G5";
                try {
                    Log.d(TAG,"onCreate: onClick: Message written");
                    write(s);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    //Should be its own thread
    //=================================================================================

    //REFERENCE: https://stackoverflow.com/questions/22899475/android-sample-bluetooth-code-to-send-a-simple-string-via-bluetooth
    private void init() throws IOException {
        Log.d(TAG,"init: called");
        BluetoothAdapter blueAdapter = BluetoothAdapter.getDefaultAdapter();
        if (blueAdapter != null) {
            Log.d(TAG,"init: BT adapter not null");
            if (blueAdapter.isEnabled()) {
                Log.d(TAG,"init: BT adapter is enabled");
                Set<BluetoothDevice> bondedDevices = blueAdapter.getBondedDevices();

                if(bondedDevices.size() > 0) {
                    Log.d(TAG,"init: Bonded devices greater than zero");
                    Object[] devices = (Object []) bondedDevices.toArray();
                    BluetoothDevice device = (BluetoothDevice) devices[position];
                    ParcelUuid[] uuids = device.getUuids();
                    BluetoothSocket socket = device.createRfcommSocketToServiceRecord(uuids[0].getUuid());
                    socket.connect();
                    outputStream = socket.getOutputStream();
                    inStream = socket.getInputStream();
                }
                Log.e("error", "No appropriate paired devices.");
            } else {
                Log.e("error", "Bluetooth is disabled.");
            }
        }
    }

    //=================================================================================

    public void write(String s) throws IOException {
        Log.d(TAG,"Write: Message sent");
        outputStream.write(s.getBytes());
    }

    //May need this later
   /* //=================================================================================

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