/*
 * Copyright 2009 Cedric Priscal
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

package android.spiport;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import android.util.Log;

public class SpiPort {

	private static final String TAG = "SpiPort";

	/*
	 * Do not remove or rename the field mFd: it is used by native method
	 * close();
	 */
	private FileDescriptor mFd;
	private FileInputStream mFileInputStream;
	private FileOutputStream mFileOutputStream;

	public SpiPort(File device, int flags) throws SecurityException,
			IOException {
		
		/* Check access permission */
		if (!device.canRead() || !device.canWrite()) {
			try {
				/* Missing read/write permission, trying to chmod the file */
				Process su;
				su = Runtime.getRuntime().exec("/system/bin/su");
				String cmd = "chmod 777 " + device.getAbsolutePath() + "\n"
						+ "exit\n";
				su.getOutputStream().write(cmd.getBytes());
				if ((su.waitFor() != 0) || !device.canRead()
						|| !device.canWrite()) {
					throw new SecurityException();
				}
			} catch (Exception e) {
				e.printStackTrace();
				throw new SecurityException();
			}
		}
		
			mFd = open(device.getAbsolutePath(), flags);
			if (mFd == null) {
				Log.e(TAG, "native open returns null");
				throw new IOException();
			}
			mFileInputStream = new FileInputStream(mFd);
			mFileOutputStream = new FileOutputStream(mFd);
	}

	/**
	 * @param filename
	 */
	public static boolean Get_Wirte_Permission(String filename) {
		Process process = null;
		DataOutputStream os = null;
		try {
			Process su;
			su = Runtime.getRuntime().exec("su");
			String cmd = "chmod 666 " + filename + "\n" + "exit\n";
			su.getOutputStream().write(cmd.getBytes());
			return true;
		} catch (Exception e) {
			Log.d("gg", "设置权限异常=" + e.getMessage());
			return false;
		}
	}

	public InputStream getInputStream() {
		return mFileInputStream;
	}

	public OutputStream getOutputStream() {
		return mFileOutputStream;
	}

	public void close_spi() {
		if (mFileInputStream != null && mFileOutputStream != null)
			close();
	}

	// JNI
	private native static FileDescriptor open(String path, int flags);

	public native void close();

	public native void up_cline();

	public native void down_cline();

	public native void write(byte[] data, int len);

	public native byte[] read(int len);

	public native byte[] read_len(int len, int[] ret_len);

	public native int Size_len(int timeout);

	public native int len(byte[] data, String str, int s);

	static {
		System.loadLibrary("serialport");
	}

}
