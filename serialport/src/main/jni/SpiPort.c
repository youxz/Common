/*
 * Copyright 2009-2011 Cedric Priscal
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <jni.h>

#include "SpiPort.h"

#include "android/log.h"
static const char *TAG="spi_port";
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)
int fd;

/*
 * Class:     android_serialport_SerialPort
 * Method:    open
 * Signature: (Ljava/lang/String;II)Ljava/io/FileDescriptor;
 */
JNIEXPORT jobject JNICALL Java_android_spiport_SpiPort_open
  (JNIEnv *env, jclass thiz, jstring path, jint flags)
{

	jobject mFileDescriptor;

	/* Opening device */
	{
		jboolean iscopy;
		const char *path_utf = (*env)->GetStringUTFChars(env, path, &iscopy);
		LOGD("Opening serial port %s with flags 0x%x", path_utf, O_RDWR | flags);
		fd = open(path_utf, O_RDWR | flags);
		LOGD("open() fd = %d", fd);
		(*env)->ReleaseStringUTFChars(env, path, path_utf);
		if (fd == -1)
		{
			/* Throw an exception */
			LOGE("Cannot open port");
			/* TODO: throw an exception */
			return NULL;
		}
	}

	/* Create a corresponding file descriptor */
	{
		jclass cFileDescriptor = (*env)->FindClass(env, "java/io/FileDescriptor");
		jmethodID iFileDescriptor = (*env)->GetMethodID(env, cFileDescriptor, "<init>", "()V");
		jfieldID descriptorID = (*env)->GetFieldID(env, cFileDescriptor, "descriptor", "I");
		mFileDescriptor = (*env)->NewObject(env, cFileDescriptor, iFileDescriptor);
		(*env)->SetIntField(env, mFileDescriptor, descriptorID, (jint)fd);
	}

	return mFileDescriptor;
}
/*
 * Class:     android_spiport_api_SpiPort
 * Method:    up_cline
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_android_spiport_SpiPort_up_1cline
  (JNIEnv *env, jobject thiz){
	int ret = ioctl(fd,0);
	  if (ret < 0)
		  LOGD("can't up_cline device\n");
	    else
	    	LOGD("up_cline device success\n");
}

/*
 * Class:     android_spiport_api_SpiPort
 * Method:    down_cline
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_android_spiport_SpiPort_down_1cline
  (JNIEnv *env, jobject thiz){
	 int ret = ioctl(fd,1);
	 if (ret < 0)
			  LOGD("can't down_cline device\n");
		    else
		    	LOGD("down_cline device success\n");
}
/*
 * Class:     cedric_serial_SerialPort
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_android_spiport_SpiPort_close
  (JNIEnv *env, jobject thiz)
{
	jclass SerialPortClass = (*env)->GetObjectClass(env, thiz);
	jclass FileDescriptorClass = (*env)->FindClass(env, "java/io/FileDescriptor");

	jfieldID mFdID = (*env)->GetFieldID(env, SerialPortClass, "mFd", "Ljava/io/FileDescriptor;");
	jfieldID descriptorID = (*env)->GetFieldID(env, FileDescriptorClass, "descriptor", "I");

	jobject mFd = (*env)->GetObjectField(env, thiz, mFdID);
	jint descriptor = (*env)->GetIntField(env, mFd, descriptorID);

	LOGD("close(fd = %d)", descriptor);

	close(descriptor);
	/*LOGD("close(fd = %d)", fd);
	close(fd);*/
}
/*
 * Class:     android_spiport_api_SpiPort
 * Method:    write
 * Signature: ([B)V
 */
JNIEXPORT void JNICALL Java_android_spiport_SpiPort_write
  (JNIEnv *env, jobject thiz, jbyteArray jOutput,jint len){
	unsigned char * output = (unsigned char *)(*env)-> GetByteArrayElements(env,jOutput, 0);//unsigned char *GetByteArrayElements
	unsigned char output_data[512];
	memset(output_data,0,512);
	memcpy(output_data,output,len);
	int ret=write(fd,output_data,512);
	(*env)->ReleaseByteArrayElements(env,jOutput, output, 0);

}
/*
 * Class:     android_spiport_api_SpiPort
 * Method:    Size_len
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_android_spiport_SpiPort_Size_1len
  (JNIEnv *env, jobject thiz, jint timeout){
	int ret=poll(fd,timeout);
	return ret;

}
/*
 * Class:     android_spiport_api_SpiPort
 * Method:    read  JNIEXPORT jint JNICALL Java_android_1spiport_1api_SpiPort_read
  (JNIEnv *, jobject, jbyteArray, jint);
 * Signature: ()[B
 */
JNIEXPORT jbyteArray JNICALL Java_android_spiport_SpiPort_read
  (JNIEnv *env, jobject thiz,jint len){
	unsigned char receive_data[len];
	memset(receive_data,0,len);
	read(fd,receive_data,len);
	jbyte *by = (jbyte*)receive_data;
	jbyteArray jarray = (*env)->NewByteArray(env,len);
	(*env)->SetByteArrayRegion(env, jarray, 0,len, by);
	return jarray;
}
/*
 * Class:     android_spiport_api_SpiPort
 * Method:    read_len
 * Signature: ([BI)I
 */
JNIEXPORT jbyteArray JNICALL Java_android_spiport_SpiPort_read_1len
  (JNIEnv *env, jobject thiz,  jint len,jintArray arr){
	unsigned char receive_data[len];
	memset(receive_data,0,len);
	jint* buf;
	buf[0]=read(fd,receive_data,len);
	(*env)->GetIntArrayRegion(env, arr, 0, 1, buf);
	jbyte *by = (jbyte*)receive_data;
	jbyteArray jarray = (*env)->NewByteArray(env,len+1);
	(*env)->SetByteArrayRegion(env, jarray, 0,1, buf);
	(*env)->SetByteArrayRegion(env, jarray, 1,len, by);
	return jarray;
}


