package android;

import java.io.InputStream;
import java.io.OutputStream;

/**
 * Created by youxz on 2017/5/3.
 */

public class JavaIO {
    private native void setInputStream(InputStream is);

    private native void setOutputStream(OutputStream out);
}
