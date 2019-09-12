package com.guoguang.common;



import android.util.Log;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

import io.reactivex.Scheduler;
import io.reactivex.annotations.NonNull;
import io.reactivex.functions.Consumer;
import io.reactivex.schedulers.Schedulers;
import io.reactivex.subjects.PublishSubject;
import io.reactivex.subjects.Subject;

/**
 * @author youxz
 * @description RxBus
 * @date 2019-09-12
 */
public class RxBus {

    private static volatile RxBus rxBus;

    private static Hashtable<Object, List<RegEventData>> subjectMapper = new Hashtable();

    public static RxBus getInstance() {
        if(rxBus == null)
            synchronized (RxBus.class){
                if(rxBus == null){
                    rxBus = new RxBus();
                }

            }
        return  rxBus;
    }


    public <T> void registerEvent(@NonNull Class<T>  eventType,@NonNull final RxBusEvent<T> obj){
        this.registerEvent(eventType,obj, Schedulers.newThread());
    }

    public <T> void registerEvent(@NonNull Class<T> eventType,@NonNull final RxBusEvent<T> obj,@NonNull Scheduler scheduler){
        Subject<Object> subject = PublishSubject.create().toSerialized();
        subject.subscribeOn(scheduler).subscribe(new Consumer<Object>() {
            @Override
            public void accept(Object o) throws Exception {
                obj.onEvent((T)o);

            }
        }, new Consumer<Throwable>() {
            @Override
            public void accept(Throwable throwable) throws Exception {
                obj.onError(throwable);
            }
        });

        List<RegEventData> list = subjectMapper.get(eventType);
        if(list == null){
           list = new ArrayList<RegEventData>();
            subjectMapper.put(eventType,list);
        }
        synchronized (list) {
            list.add(new RegEventData(subject,obj));
        }

    }

    public void unRegisterEvent(@NonNull Object eventType){
        List<RegEventData> list = subjectMapper.remove(eventType);
        if (list!=null){
            synchronized (list) {
                for (int i = list.size() - 1; i >= 0; i--) {
                    list.remove(i).getSubject().onComplete();
                }
            }
        }

    }

    public <T>void unRegisterEvent(@NonNull Class<T> eventType,@NonNull RxBusEvent<T> obj){
        List<RegEventData> list = subjectMapper.get(eventType);
        synchronized (list) {
            if (list != null) {
                for (int i = list.size() - 1; i >= 0; i--) {
                    if (list.get(i).getObject().equals(obj)) {
                        list.remove(i).getSubject().onComplete();
                    }
                }
            }
            if (list.size() == 0) {
                subjectMapper.remove(eventType);
            }
        }

    }

    public void postEvent(@NonNull Object eventObj){
        List<RegEventData> list = subjectMapper.get(eventObj.getClass());
        if(list != null){
            for (RegEventData item:list){
                item.getSubject().onNext(eventObj);
            }
        }
    }


    public static abstract class RxBusEvent<T>{
       abstract void  onEvent(T event);
       void onError(Throwable e){
           Log.e("RxBus","Error",e);
        }
    }



    private class RegEventData{
        private Subject<Object> subject;
        private Object object;

        RegEventData(Subject<Object> subject,Object obj){
            this.subject = subject;
            this.object = obj;
        }

        public Subject<Object> getSubject() {
            return subject;
        }

        public Object getObject() {
            return object;
        }

    }




}
