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
#include <sys/ioctl.h>

#include "SerialPort.h"

#include "android/log.h"
static const char *TAG="serial_port";
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)
int fd;

static speed_t getBaudrate(jint baudrate)
{
	switch(baudrate) {
		case 0: return B0;
		case 50: return B50;
		case 75: return B75;
		case 110: return B110;
		case 134: return B134;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
		case 1800: return B1800;
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		case 500000: return B500000;
		case 576000: return B576000;
		case 921600: return B921600;
		case 1000000: return B1000000;
		case 1152000: return B1152000;
		case 1500000: return B1500000;
		case 2000000: return B2000000;
		case 2500000: return B2500000;
		case 3000000: return B3000000;
		case 3500000: return B3500000;
		case 4000000: return B4000000;
		default: return -1;
	}
}
/*
 * Class:     android_serialport_SerialPort
 * Method:    open
 * Signature: (Ljava/lang/String;II)Ljava/io/FileDescriptor;
 */
JNIEXPORT jobject JNICALL Java_android_serialport_SerialPort_open
  (JNIEnv *env, jclass thiz, jstring path, jint baudrate,jint parity)
{

	jobject mFileDescriptor;
    speed_t speed;
	/* Check arguments */
	{
		speed = getBaudrate(baudrate);
		if (speed == -1) {
			/* TODO: throw an exception */
			LOGE("Invalid baudrate");
			return NULL;
		}
	}

	/* Opening device */
	{
		jboolean iscopy;
		const char *path_utf = (*env)->GetStringUTFChars(env, path, &iscopy);
		LOGD("Opening serial port %s with baudrate %d  parity:%c", path_utf,  baudrate,parity);
		//fd = open(path_utf, O_RDWR | 0x2);
        fd = open(path_utf, O_RDWR|O_NOCTTY);//O_NDELAY O_NOCTTY
        set_Parity(fd,8,1,(char)parity,speed,10);
       // ioctl(fd,SIO_HW_OPTS_SET, CS8|PARENB|CLOCAL|CREAD);
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

int set_Parity(int fd,int databits,int stopbits,int parity, int baudrate, int timeout)
{
    struct termios options;
    if ( tcgetattr( fd,&options) != 0) {
        LOGE("SetupSerial 1");
        return -1;
    }
    options.c_cflag |=(CLOCAL|CREAD);
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            LOGE("Unsupported data sizen"); return -1;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; /* Clear parity enable */
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            options.c_iflag |= INPCK; /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB; /* Enable parity */
            options.c_cflag &= ~PARODD; /* 转换为偶效验*/
            options.c_iflag |= INPCK; /* Disnable parity checking */
            break;
        case 'S':
        case 's': /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            LOGE("Unsupported parityn");
            return -1;
    }

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            LOGE("Unsupported stop bitsn");
            return -1;
    }

    options.c_lflag&=~(ICANON|ECHO|ECHOE|ISIG);
    options.c_oflag&=~OPOST;
    options.c_oflag&=~(ONLCR|OCRNL);
    options.c_iflag&=~(ICRNL|INLCR);
    options.c_iflag&=~(IXON|IXOFF|IXANY);

    /* 设置超时 10 seconds*/
    options.c_cc[VTIME] = (timeout <= 0 ? 10:timeout);

    options.c_cc[VMIN]  = 0;  /* Update the options and do it NOW */

    tcflush(fd,TCIFLUSH);

    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        LOGE("SetupSerial 3");

        return -1;
    }
    return 1;
}

/*
 * Class:     android_spiport_api_SpiPort
 * Method:    up_cline
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_android_serialport_SerialPort_up_1cline
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
JNIEXPORT void JNICALL Java_android_serialport_SerialPort_down_1cline
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
JNIEXPORT void JNICALL Java_android_serialport_SerialPort_close
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
JNIEXPORT void JNICALL Java_android_1spiport_1api_SpiPort_write
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
JNIEXPORT jint JNICALL Java_android_1spiport_1api_SpiPort_Size_1len
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
JNIEXPORT jbyteArray JNICALL Java_android_1spiport_1api_SpiPort_read
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
JNIEXPORT jbyteArray JNICALL Java_android_1spiport_1api_SpiPort_read_1len
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


