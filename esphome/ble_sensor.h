#include "esphome.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "ArduinoJson.h"
#include "BLEBeacon.h"
#include "BLEEddystoneTLM.h"
#include "BLEEddystoneURL.h"


#define singleScanTime 5

#define maxDistance 3

#define activeScan true // Active scan uses more power, but get results faster
#define bleScanInterval 0x80 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing
#define bleScanWindow 0x10 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing

uint16_t beaconUUID = 0xFEAA;
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

const int MAX_BEACON_COUNT = 4;

class KalmanFilter {
    private:
        float _pR; // noise power desirable
        float _pQ; // noise power estimated
        float _pA;
        float _pC;
        float _pB;
        float _pCOV;
        int _pX; // estimated signal without noise      

    public:
        KalmanFilter(float r = 1, float q = 1, float a = 1, float b = 0, float c = 1){
            _pR = r; // noise power desirable
            _pQ = q; // noise power estimated
            _pA = a;
            _pC = c;
            _pB = b;
            _pCOV = 0;
            _pX = 0; // estimated signal without noise
        }

        float filter(int z, int u = 0){
            if (_pX == 0) {
                _pX = round(((1 / _pC) * (float)z));
                _pCOV = (1 / _pC) * _pQ * (1 / _pC);
            }else {
                // Compute prediction
                const float predX = predict(u);
                const float predCov = uncertainty();
                // Kalman gain
                const float K = predCov * _pC * (1 / ((_pC * predCov * _pC) + _pQ));
                // Correction
                _pX = round(predX + K * ((float)z - (_pC * predX)));
                _pCOV = predCov - (K * _pC * predCov);
            }
            return _pX;
        }

        float predict(int u = 0){
            return (_pA * (float)_pX) + (_pB * (float)u);
        }

        float uncertainty(){
            return ((_pA * _pCOV) * _pA) + _pR;
        }

        float lastMeasurement(){
            return _pX;
        }

        void setMeasurementNoise(float noise){
            _pQ = noise;
        }

        void setProcessNoise(float noise) {
            _pR = noise;
        }
};

class BeaconClass {
	public:
		KalmanFilter kfilter = KalmanFilter();
		int seenCount;

		BeaconClass(){
			seenCount = 0;
		}
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

	void onResult(BLEAdvertisedDevice advertisedDevice) {

        ESP_LOGD("ble_sensor.advertisedDevice", "Advertised Device: %s", advertisedDevice.toString().c_str());

	}
};

bool isGreaterThanMax(BeaconClass val)
{
if(val.seenCount > MAX_BEACON_COUNT)
	return true;
else
	return false;
}

class BLESensor : public PollingComponent, public Sensor {
    public:
    
    int scanTime;
    // constructor
    BLESensor(int interval) : PollingComponent(interval) {}

    void setup() override {
        // This will be called by App.setup()
        scanTime = singleScanTime;

        BLEDevice::init("");
        pBLEScan = BLEDevice::getScan(); //create new scan
        pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
        pBLEScan->setActiveScan(activeScan);
        pBLEScan->setInterval(bleScanInterval);
        pBLEScan->setWindow(bleScanWindow);
    }
    void update() override {
        // This will be called every "update_interval" milliseconds.
        std::vector<String> beaconsVector(0);
        
        ESP_LOGI("ble_sensor", "Scanning...");

        BLEScanResults foundDevices = pBLEScan->start(scanTime);
        devicesCount = foundDevices.getCount();
        ESP_LOGI("ble_sensor", "Scan done! Devices found: %d",devicesCount);

        int devicesReported = 0;
        for (uint32_t i = 0; i < devicesCount; i++) {
            bool included = reportDevice(foundDevices.getDevice(i), beaconsVector);
            if (included) {
                devicesReported++;
            }
        }

        countBeacons(beaconsVector);

        ESP_LOGI("ble_sensor", "Devices reported: %d \n", devicesReported);
        pBLEScan->clearResults();
        int beaconsDeleted = deleteLostBeacons(beaconsDict, &isGreaterThanMax);
    }
    
    private:
        BLEScan* pBLEScan;
        int devicesCount = 0;
        std::map<String, BeaconClass> beaconsDict;

        const int BEACON_PLUS_POWER = -13;
        
        void countBeacons(std::vector<String>& beaconsVector){

            bool itemFound = false;
            for(auto& dict : beaconsDict)
            {
                for(String& itemVector : beaconsVector)
                {
                    if (dict.first == itemVector){
                        itemFound = true;
                        break;
                    }
                }
                if (!itemFound) dict.second.seenCount += 1; 
            }
        }

        template<typename  K, typename  V>
        int deleteLostBeacons(std::map<K, V> & mapOfElemen, bool(* functor)(V))
        {
            int totalDeletedElements = 0;
            auto it = mapOfElemen.begin();
            while(it != mapOfElemen.end())
            {
                if(functor(it->second))
                {
                    totalDeletedElements++;
                    it = mapOfElemen.erase(it);
                }
                else
                {
                    it++;
                }
            }
            return totalDeletedElements;
        }

        String getProximityUUIDString(BLEBeacon beacon) {
            std::string serviceData = beacon.getProximityUUID().toString().c_str();
            int serviceDataLength = serviceData.length();
            String returnedString = "";
            int i = serviceDataLength;
            while (i > 0)
            {
                if (serviceData[i-1] == '-') {
                    i--;
                }

                char a = serviceData[i-1];
                char b = serviceData[i-2];
                returnedString += b;
                returnedString += a;

                i -= 2;
            }

            return returnedString;
        }

        float calculateDistance(int rssi, int txPower, String macAddress, int n = 2, int d0 = 1) {

            BeaconClass beaconData;

            if (rssi == 0) {
                return -59;
            }

            if (!txPower) {
                // somewhat reasonable default value
                txPower = -59;
            } else if (txPower > 0) {
                txPower = txPower * -1;
            }

            if (beaconsDict.count(macAddress) == 1){
                beaconData = beaconsDict[macAddress];
                beaconData.seenCount = 0;
            }else{
                KalmanFilter k = KalmanFilter(1, 1, 1, 0, 1);
                BeaconClass beaconData = BeaconClass();
                beaconData.kfilter = k;
            }
            int rssiFiltered = beaconData.kfilter.filter(rssi);
            beaconsDict[macAddress] = beaconData;

            //float dist = powf(((float)rssi - (float)txPower) / (10 * (float)n), 10.0) * (float)d0;
            float dist = powf(10.0, ((float)txPower - (float)rssiFiltered) / (10 * (float)n)) * (float)d0;

            return round(dist * 100) / 100;
        }

        bool reportDevice(BLEAdvertisedDevice advertisedDevice, std::vector<String>& beaconsVector) {

            StaticJsonDocument<500> doc;

            String mac_address = advertisedDevice.getAddress().toString().c_str();
            mac_address.replace(":","");
            mac_address.toLowerCase();

            int rssi = advertisedDevice.getRSSI();
            float distance = 100.0;

            doc["id"] = mac_address;
            doc["uuid"] = mac_address;
            doc["rssi"] = rssi;

            beaconsVector.push_back(mac_address);

            if (advertisedDevice.haveName()){
                String nameBLE = String(advertisedDevice.getName().c_str());
                doc["name"] = nameBLE;
            }else{
                doc["name"] = "unknown";
            }

            std::string strServiceData = advertisedDevice.getServiceData();
            uint8_t cServiceData[100];
            strServiceData.copy((char *)cServiceData, strServiceData.length(), 0);

            if (advertisedDevice.getServiceDataUUID().equals(BLEUUID(beaconUUID))==true){
                // found Eddystone UUID
            }else{
                if (advertisedDevice.haveManufacturerData()==true) {
                    std::string strManufacturerData = advertisedDevice.getManufacturerData();
                    uint8_t cManufacturerData[100];
			        strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);

                    if (strManufacturerData.length()==25 && cManufacturerData[0] == 0x4C  && cManufacturerData[1] == 0x00 ) {
                        BLEBeacon oBeacon = BLEBeacon();
                        oBeacon.setData(strManufacturerData);

                        String proximityUUID = getProximityUUIDString(oBeacon);

                        int A0 = oBeacon.getSignalPower() + BEACON_PLUS_POWER;
                        distance = calculateDistance(rssi, A0, mac_address);

                        int major = ENDIAN_CHANGE_U16(oBeacon.getMajor());
				        int minor = ENDIAN_CHANGE_U16(oBeacon.getMinor());

                        doc["major"] = major;
                        doc["minor"] = minor;

                        doc["uuid"] = proximityUUID;
                        doc["id"] = proximityUUID + "-" + String(major) + "-" + String(minor);
                        doc["txPower"] = A0;
                        doc["distance"] = distance;
                    }else{
                        if (advertisedDevice.haveTXPower()) {
                            distance = calculateDistance(rssi, advertisedDevice.getTXPower(), mac_address);
                            doc["txPower"] = advertisedDevice.getTXPower();
                        } else {
                            distance = calculateDistance(rssi, -59, mac_address);
                        }
                        doc["distance"] = distance;
                    }
                }else{
                    if (advertisedDevice.haveTXPower()) {
                        distance = calculateDistance(rssi, advertisedDevice.getTXPower(), mac_address);
                        doc["txPower"] = advertisedDevice.getTXPower();
                        doc["distance"] = distance;
                    }else{
                        distance = calculateDistance(rssi, -59, mac_address);
                        doc["txPower"] = -59;
                        doc["distance"] = distance;
                    }
                }
            }

            const char* did = doc["id"];
            const char* duuid = doc["uuid"];
            int drssi = doc["rssi"];
            const char* dname = doc["name"];
            int dtxPower = doc["txPower"];
            float ddistance = doc["distance"];

            ESP_LOGD("ble_sensor.reportDevice", "id: %s", did);
            ESP_LOGD("ble_sensor.reportDevice", "uuid: %s", duuid);
            ESP_LOGD("ble_sensor.reportDevice", "rssi: %d", drssi);
            ESP_LOGD("ble_sensor.reportDevice", "name: %s", dname);
            ESP_LOGD("ble_sensor.reportDevice", "txPower: %d", dtxPower);
            ESP_LOGD("ble_sensor.reportDevice", "distance: %.2f\n", ddistance);

            if (doc["distance"] < maxDistance) {
                return true;
            }

            return false;
        }
};