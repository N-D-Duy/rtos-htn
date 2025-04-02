#include "service.h"
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <time.h>
#include "config.h"

// WiFi credentials
const char *ssid = "Realme GT Neo2 5G";
const char *password = "Duy9999***#";
#define WIFI_CHANNEL 6

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
String streamPath = "/iot";

Service::Service()
{
    Serial.begin(115200);
}

void Service::syncFirebaseData()
{
    FirebaseData fbdo;
    FirebaseJson jsonData;
    if (Firebase.RTDB.getJSON(&fbdo, streamPath.c_str()))
    {
        if(fbdo.dataType() != "null"){
            
            jsonData.setJsonData(fbdo.jsonString());
            FirebaseJsonData result;
            if(jsonData.get(result, "door") && result.type == "boolean"){
                _doorStatus = result.boolValue;
            } 

            if(jsonData.get(result, "bell") && result.type == "boolean"){
                _ringStatus = result.boolValue;
            }
            Serial.printf("Door status: %d\n, Ring status: %d\n", _doorStatus, _ringStatus);
        }
    }
    else
    {
        Serial.println(fbdo.errorReason());
    }
}

void Service::initService()
{
    Serial.println("Initializing service...");
    if(API_KEY == "" || DATABASE_URL == "" || USER_EMAIL == "" || USER_PASSWORD == "") {
        Serial.println("Credentials is not set!");
        return;
    }
    int wifiTimeout = 30;
    WiFi.begin(ssid, password, WIFI_CHANNEL);

    while (WiFi.status() != WL_CONNECTED && wifiTimeout > 0) {
        delay(500);
        Serial.print(".");
        wifiTimeout--;
    }
    Serial.println("\nConnected to WiFi");

    // Firebase configuration
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.database_url = DATABASE_URL;
    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);
    config.token_status_callback = tokenStatusCallback;
    config.max_token_generation_retry = 5;

    Firebase.begin(&config, &auth);

    configTime(0, 0, "pool.ntp.org");
    delay(2000);
}

boolean Service::setDoorStatus(bool status)
{
    _doorStatus = status;
    FirebaseJson json;
    json.add("door", status);
    if(Firebase.RTDB.updateNode(&fbdo, streamPath.c_str(), &json)){
        // Serial.println("data is updated via firebase");
    } else {
        Serial.println(fbdo.errorReason());
        return false;
    }
    return true;
}

boolean Service::getDoorStatus()
{
    return _doorStatus;
}

boolean Service::setRingStatus(bool status)
{
    _ringStatus = status;
    FirebaseJson json;
    json.add("bell", status);
    if(Firebase.RTDB.updateNode(&fbdo, streamPath.c_str(), &json)){
        // Serial.println("data is updated via firebase");
    } else {
        Serial.println(fbdo.errorReason());
        return false;
    }
    return true;
}

