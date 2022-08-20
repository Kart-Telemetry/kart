#include <LoRa.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include "boards.h"

TinyGPSPlus gps;

unsigned long last = 0UL;

byte localAddress = 0x10; // address of this device
byte destination = 0x20;  // destination to send to
byte broadcast = 0xFF;

void sendStringMessage(String outgoing)
{
    LoRa.beginPacket();            // start packet
    LoRa.write(destination);       // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it

    LoRa.receive();

    Serial.println("Transmitted message");
    Serial.println("Sent to: 0x" + String(destination, HEX));
    Serial.println("Message length: " + String(outgoing.length()));
    Serial.println("Message: " + outgoing);
    Serial.println();
}

void sendMessage(DynamicJsonDocument doc)
{
    String output;
    serializeJson(doc, output);
    sendStringMessage(output);
}

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }

    Serial.println("LoRa init succeeded.");
}

void loop()
{
    // This sketch displays information every time a new sentence is correctly encoded.
    while (Serial1.available() > 0)
        gps.encode(Serial1.read());

    if (millis() - last > 1000)
    {
        DynamicJsonDocument doc(1024);
        doc["lat"] = gps.location.lat();
        doc["lng"] = gps.location.lng();
        char buffer[24];
        sprintf(buffer, "%02d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
        doc["timestamp"] = String(buffer);
        doc["speed"] = gps.speed.kmph();
        doc["course"] = gps.course.deg();
        doc["altitude"] = gps.altitude.meters();
        doc["satellites"] = gps.satellites.value();
        doc["hdop"] = gps.hdop.hdop();
        doc["battery"] = PMU.getBattVoltage() / 1000;
        sendMessage(doc);

        if (gps.charsProcessed() < 10)
            Serial.println(F("WARNING: No GPS data.  Check wiring."));

        last = millis();
        Serial.println();
    }

    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        // read packet header bytes:
        int recipient = LoRa.read();       // recipient address
        byte sender = LoRa.read();         // sender address
        byte incomingLength = LoRa.read(); // incoming msg length

        String incoming = ""; // payload of packet

        while (LoRa.available())
        {                                  // can't use readString() in callback, so
            incoming += (char)LoRa.read(); // add bytes one by one
        }

        if (incomingLength != incoming.length())
        { // check length for error
            Serial.println("error: message length does not match length");
            return; // skip rest of function
        }

        Serial.println("Received from: 0x" + String(sender, HEX));

        // if the recipient isn't this device or broadcast,
        if (recipient != localAddress && recipient != broadcast)
        {
            Serial.println("This message is not for me.");
            return; // skip rest of function
        }

        // if message is for this device, or broadcast, print details:
        Serial.println("Sent to: 0x" + String(recipient, HEX));
        Serial.println("Message length: " + String(incomingLength));
        Serial.println("Message: " + incoming);
        Serial.println("RSSI: " + String(LoRa.packetRssi()));
        Serial.println();

        DynamicJsonDocument idoc(1024);
        deserializeJson(idoc, incoming);

        if (idoc["type"] == "command")
        {
            DynamicJsonDocument doc(1024);
            doc["type"] = "command_ack";
            doc["content"] = idoc;
            sendMessage(doc);
        }
    }
}
