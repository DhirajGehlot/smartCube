/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

static BLEAddress *pServerAddress = new BLEAddress("CD:59:03:D3:8F:D0");

BLEServer *pServer = NULL;
// The remote service we wish to connect to
static BLEUUID serviceUUID("0000aadb-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we want to track
static BLEUUID charUUID("0000aadc-0000-1000-8000-00805f9b34fb");
// The following constants are used to decrypt the data representing the state of the cube
// see https://github.com/cs0x7f/cstimer/blob/master/src/js/bluetooth.js
const uint8_t decryptionKey[] = {176, 81, 104, 224, 86, 137, 237, 119, 38, 26, 193, 161, 210, 126, 150, 81, 93, 13, 236, 249, 89, 235, 88, 24, 113, 81, 214, 131, 130, 199, 2, 169, 39, 165, 171, 41};
// This pin will have a HIGH pulse sent when the cube is solved
const byte relayPin = 0;
// This is the data array representing a solved cube
const byte solution[16] = {0x12,0x34,0x56,0x78,0x33,0x33,0x33,0x33,0x12,0x34,0x56,0x78,0x9a,0xbc,0x00,0x00};

// GLOBALS
// Have we found a cube with the right MAC address to connect to?
static boolean deviceFound = false;
// Are we currently connected to the cube?
static boolean connected = false;
// Properties of the device found via scan
static BLEAdvertisedDevice* myDevice;
// BT characteristic of the connected device
static BLERemoteCharacteristic* pRemoteCharacteristic;

int getBit(uint8_t* val, int i) {
  int n = ((i / 8) | 0);
  int shift = 7 - (i % 8);
  return (val[n] >> shift) & 1;    
}

/**
 * Return the ith nibble (half-byte, i.e. 16 possible values)
 */
uint8_t getNibble(uint8_t val[], int i) {
  if(i % 2 == 1) {
    return val[(i/2)|0] % 16;
  }
  return 0|(val[(i/2)|0] / 16);
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  // The onResult callback is called for every advertised device found by the scan  
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Print the MAC address of this device
    Serial.print(" - ");
    Serial.print(advertisedDevice.getAddress().toString().c_str());
    // Does this device match the MAC address we're looking for?
    if(advertisedDevice.getAddress().equals(*pServerAddress)) {
      // Stop scanning for further devices
      advertisedDevice.getScan()->stop();
      // Create a new device based on properties of advertised device
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      // Set flag
      deviceFound = true;
      Serial.println(F(" - Connecting!"));
    }
    else {
      Serial.println(F("... MAC address does not match"));
    }
  }
};

/**
 * Callbacks for device we connect to
 */
class ClientCallbacks : public BLEClientCallbacks {
  // Called when a new connection is established
  void onConnect(BLEClient* pclient) {
    //digitalWrite(LED_BUILTIN, HIGH);
    connected = true;
  }
  // Called when a connection is lost
  void onDisconnect(BLEClient* pclient) {
    //digitalWrite(LED_BUILTIN, LOW);
    connected = false;
  }
};

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // DECRYPT DATA
  // Early Bluetooth cubes used an unencrypted data format that sent the corner/edge indexes as a 
  // simple raw string, e.g. pData for a solved cube would be 1234567833333333123456789abc000041414141
  // However, newer cubes e.g. Giiker i3s encrypt data with a rotating key, so that same state might be
  // 706f6936b1edd1b5e00264d099a4e8a19d3ea7f1 then d9f67772c3e9a5ea6e84447abb527156f9dca705 etc.
  
  // To find out whether the data is encrypted, we first read the penultimate byte of the characteristic data. 
  // As in the two examples above, if this is 0xA7, we know it's encrypted
  bool isEncrypted = (pData[18] == 0xA7);

  // If it *is* encrypted...
  if(isEncrypted) {
    // Split the last byte into two 4-bit values 
    int offset1 = getNibble(pData, 38);
    int offset2 = getNibble(pData, 39);

    // Retrieve a pair of offset values from the decryption key 
    for (int i=0; i<20; i++) {
      // Apply the offset to each value in the data
      pData[i] += (decryptionKey[offset1 + i] + decryptionKey[offset2 + i]);
    }
  }

  // First 16 bytes represent state of the cube - 8 corners (with 3 orientations), and 12 edges (can be flipped)
  Serial.print("Current State: ");
    for (int i=0; i<16; i++) {
      Serial.print(pData[i], HEX);
      Serial.print(" ");
    }
  Serial.println("");

  // Byte 17 represents the last twist made - first half-byte is face, and second half-byte is direction of rotation
  int lastMoveFace = getNibble(pData, 32);
  int lastMoveDirection = getNibble(pData, 33);
  char* faceNames[6] = {"Front", "Bottom", "Right", "Top", "Left", "Back"};
  Serial.print("Last Move: ");
  Serial.print(faceNames[lastMoveFace-1]);
  Serial.print(lastMoveDirection == 1 ? " Face Clockwise" : " Face Anti-Clockwise" );
  Serial.println("");
  
  Serial.println("----");

  if(memcmp(pData, solution, 16) == 0) {
    digitalWrite(relayPin, HIGH);
    delay(5000);
    digitalWrite(relayPin, LOW);
    Serial.print("CUBE SOLVED !!!!!!!");
    doSomething();
  }
  
}

/*
 * Connect to the BLE server of the correct MAC address
 */
bool connectToServer() {
    Serial.print(F("Creating BLE client... "));
    BLEClient* pClient = BLEDevice::createClient();
    delay(500);
    Serial.println(F("Done."));

    Serial.print(F("Assigning callbacks... "));
    pClient->setClientCallbacks(new ClientCallbacks());
    delay(500);
    Serial.println(F(" - Done."));

    // Connect to the remove BLE Server.
    Serial.print(F("Connecting to "));
    Serial.print(myDevice->getAddress().toString().c_str());
    Serial.print(F("... "));
    pClient->connect(myDevice);
    delay(500);
    Serial.println(" - Done.");
    
    // Obtain a reference to the service we are after in the remote BLE server.
    Serial.print(F("Finding service "));
    Serial.print(serviceUUID.toString().c_str());
    Serial.print(F("... "));
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    delay(500);
    if (pRemoteService == nullptr) {
      Serial.println(F("FAILED."));
      return false;
    }
    Serial.println(" - Done.");
    delay(500);

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    Serial.print(F("Finding characteristic "));
    Serial.print(charUUID.toString().c_str());
    Serial.print(F("... "));
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.println(F("FAILED."));
      return false;
    }
    Serial.println(" - Done.");
    delay(500);
    
    Serial.print(F("Registering for notifications... "));
    if(pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);
      Serial.println(" - Done.");
    }
    else {
      Serial.println(F("FAILED."));
      return false;
    }

    Serial.println("READY!");
}

/**
 * Search for any advertised devices
 */
void scanForDevices(){
  Serial.println("Scanning for Bluetooth devices...");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}


BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};


void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if (deviceFound) {
    if(!connected) {
      connectToServer();
    }
  }
  else {
    scanForDevices();
  }
  // Introduce a little delay
  delay(1000);

  /*
    if (deviceConnected) {
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    */
}

void doSomething()
{
  
}

