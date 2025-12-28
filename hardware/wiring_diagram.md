# Wiring Diagram

## Milestone 1: Temperature Sensing Only

### DS18B20 Waterproof Sensor
```
DS18B20              Arduino Uno
┌─────────┐         ┌───────────┐
│  RED    │────────→│ 5V        │
│ (VCC)   │         │           │
│         │         │           │
│ BLACK   │────────→│ GND       │
│ (GND)   │         │           │
│         │         │           │
│ YELLOW  │────────→│ Pin 2     │
│ (DATA)  │    ┌───→│           │
└─────────┘    │    └───────────┘
               │
            4.7kΩ
          Pull-up
         Resistor
               │
              5V
```

### Parts Required:
- Arduino Uno R3
- DS18B20 waterproof temperature sensor
- 4.7kΩ resistor (brown-violet-red or yellow-violet-brown)
- Breadboard
- Jumper wires

### Notes:
- The 4.7kΩ pull-up resistor is **required** for reliable communication
- Use the resistor from your Elegoo kit
- Keep sensor wires short if possible (<1 meter)

---

## Milestone 2: Adding Heater Control (Coming Soon)

Will add SSR and heating element wiring.