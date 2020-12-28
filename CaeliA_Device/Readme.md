# Note o libraries

An issue arrived with a coliding redefinition of HTTP_HEAD in the HTTPClient and WiFiManager libraries.

The issue was solved by editing files WiFiManager.cpp and WiFiManager.h from the WiFi manager library,
substituting the HTTP_HEAD by HTTP_HEADER

    const char HTTP_HEAD[] PROGMEM
    by
    const char HTTP_HEADER[] PROGMEM
