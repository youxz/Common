package com.guoguang.common;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;

public class MainActivity extends AppCompatActivity {

    private RxBus rxBus;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        rxBus = RxBus.getInstance();


        rxBus.registerEvent(String.class,new RxBus.RxBusEventAdaptor<String>(){

            void onEvent(String event) {

                Log.i("TEVENT",event);

            }
        });

    }

    @Override
    protected void onStart() {
        super.onStart();

        rxBus.postEvent("hello 1");
        rxBus.postEvent("hello 2");
        rxBus.postEvent("hello 3");
        rxBus.postEvent("hello 4");
        rxBus.postEvent("hello 5");
        //rxBus.unRegisterEvent(String.class);
        rxBus.postEvent("hello 1");
        rxBus.postEvent("hello 2");
        rxBus.postEvent("hello 3");
        rxBus.postEvent("hello 4");
        rxBus.postEvent("hello 5");


    }
}
