# Bill of Materials (BOM)

Complete component list for the PID Temperature Controller project.

---

## Core Components

### Microcontroller

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Part Number / Link | Notes |
|------|----------|----------------|-----------|----------|-------------------|-------|
| Arduino Uno R3 | 1 | 0.00 | 0.00 | Already owned | Elegoo variant | ATmega328P, 16MHz |

**Alternative:**
- Genuine Arduino Uno R3: £22.00 - [Arduino Store](https://store.arduino.cc/products/arduino-uno-rev3)
- Elegoo Uno R3: £10.99 - [Amazon UK](https://www.amazon.co.uk/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU)

---

### Temperature Sensing

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Part Number / Link | Notes |
|------|----------|----------------|-----------|----------|-------------------|-------|
| DS18B20 Waterproof Temperature Sensor | 1 | 6.99 | 6.99 | Amazon UK | [Link](https://www.amazon.co.uk/Gikfun-DS18B20-Waterproof-Temperature-EK1083x2/dp/B012C597T0) | ±0.5°C accuracy, 1m cable |
| 4.7kΩ Resistor | 1 | 0.00 | 0.00 | Already owned | From Elegoo kit | Pull-up resistor |

**Specifications:**
- **Sensor Type:** Digital 1-Wire
- **Accuracy:** ±0.5°C (-10°C to +85°C)
- **Resolution:** 9-12 bit (programmable)
- **Response Time:** <10 seconds in water
- **Cable Length:** 1 meter (waterproof)
- **Operating Voltage:** 3.0V to 5.5V

**Alternative Sensors:**

| Sensor | Price | Temp Range | Response Time | Notes |
|--------|-------|------------|---------------|-------|
| MAX6675 + K-Type Thermocouple | £6.99 | 0-1024°C | ~1s | Faster, higher temp, SPI interface |
| DHT22 | £4.99 | -40 to 80°C | ~2s | Also measures humidity |
| BME280 | £5.99 | -40 to 85°C | ~1s | I2C, pressure + humidity |

**Recommendation:** DS18B20 - Best for water bath, simple wiring, adequate speed

---

### Power Switching

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Part Number / Link | Notes |
|------|----------|----------------|-----------|----------|-------------------|-------|
| SSR-25DA Solid State Relay | 1 | 5.99 | 5.99 | Amazon UK | [Link](https://www.amazon.co.uk/uxcell-SSR-25DA-Solid-State-Relay/dp/B00E1LC1VK) | 25A, 3-32VDC control |

**Specifications:**
- **Control Voltage:** 3-32 VDC
- **Load Voltage:** 24-380 VAC
- **Load Current:** 25A (continuous)
- **Control Current:** 7.5 mA (at 12V)
- **Switching Time:** 10ms
- **Isolation:** 2500 VAC

**Why SSR instead of mechanical relay:**
- ✅ No mechanical wear (millions of switching cycles)
- ✅ Silent operation (no clicking)
- ✅ Fast switching (10ms vs. 10-15ms)
- ✅ No electrical noise/arcing
- ❌ More expensive
- ❌ Heat dissipation (use heatsink for >10A loads)

**Alternative for DC Loads:**

| Component | Price | Current | Voltage | Notes |
|-----------|-------|---------|---------|-------|
| FR120N MOSFET Module | £3.49 | 9.4A | 100V | Cheaper for DC heating only |
| IRLZ44N MOSFET (DIY) | £1.20 | 47A | 55V | Requires gate driver circuit |

---

### Heating Element

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Part Number / Link | Notes |
|------|----------|----------------|-----------|----------|-------------------|-------|
| 12V 5W Silicone Heating Pad | 1 | 8.99 | 8.99 | Amazon UK | [Link](https://www.amazon.co.uk/Silicone-Heater-Element-Flexible-Heating/dp/B07VNXK7QF) | 50x50mm, flexible |

**Specifications:**
- **Voltage:** 12 VDC
- **Power:** 5W (nominal)
- **Current:** 0.42A at 12V
- **Size:** 50mm x 50mm
- **Max Temperature:** 100°C
- **Material:** Silicone rubber

**Heating Pad Selection Guide:**

| Power | Temp Rise (300ml water) | Time to 50°C | Overshoot Risk | Cost |
|-------|------------------------|--------------|----------------|------|
| 5W | ~30°C | 6-8 min | Low | £8.99 |
| 10W | ~50°C | 3-5 min | Medium | £10.99 |
| 25W | ~70°C | 1-3 min | High | £12.99 |

**Recommendation:** 5W - Safe, controllable, suitable for learning PID

**Alternatives:**

| Heating Method | Price | Power | Notes |
|----------------|-------|-------|-------|
| 12V Cartridge Heater (40W) | £7.99 | 40W | Faster but needs thermal management |
| Soldering Iron Element | £5.99 | 20-40W | Repurpose cheap iron, DIY solution |
| Nichrome Wire DIY | £3.00 | Custom | Requires calculation and insulation |

---

### Power Supply

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Part Number / Link | Notes |
|------|----------|----------------|-----------|----------|-------------------|-------|
| 12V 2A DC Power Adapter | 1 | 7.99 | 7.99 | Amazon UK | [Link](https://www.amazon.co.uk/SHNITPWR-Adapter-Transformers-100-240V-5-5x2-1mm/dp/B0852HL336) | 5.5×2.1mm barrel jack |

**Specifications:**
- **Input:** 100-240 VAC, 50/60Hz
- **Output:** 12 VDC, 2A (24W max)
- **Connector:** 5.5mm × 2.1mm barrel plug (center positive)
- **Regulation:** ±5%
- **Safety:** CE, RoHS certified

**Power Requirements:**
- Heating pad: 5W (0.42A)
- Arduino: ~0.5W (via USB, separate)
- **Total:** 5.5W (0.5A margin is safe)

**Alternative:** Use existing 12V laptop charger or wall adapter (check polarity!)

---

### Passive Components & Wiring

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Notes |
|------|----------|----------------|-----------|----------|-------|
| Breadboard | 1 | 0.00 | 0.00 | Already owned | From Elegoo kit |
| Jumper Wires (M-M, M-F) | 20+ | 0.00 | 0.00 | Already owned | From Elegoo kit |
| 4.7kΩ Resistor | 1 | 0.00 | 0.00 | Already owned | DS18B20 pull-up |
| LED (for status) | 1 | 0.00 | 0.00 | Already owned | Optional, for debugging |
| 220Ω Resistor (for LED) | 1 | 0.00 | 0.00 | Already owned | If using external LED |

**Notes:**
- Elegoo Super Starter Kit includes breadboard, wires, resistors, LEDs
- No additional passive components needed

---

### Container & Insulation

| Item | Quantity | Unit Price (£) | Total (£) | Supplier | Notes |
|------|----------|----------------|-----------|----------|-------|
| Plastic Food Container (300-500ml) | 1 | 1.00 | 1.00 | Local shop | Water bath vessel |
| Insulation (optional) | 1 | 2.00 | 2.00 | DIY store | Foam/bubble wrap to reduce heat loss |

**Container Requirements:**
- **Volume:** 300-500ml (larger = slower response, more stable)
- **Material:** Plastic (not metal, to avoid grounding issues)
- **Lid:** Helps retain heat and reduces evaporation
- **Sensor Access:** Hole for waterproof sensor

**Recommendation:** Use kitchen container (reusable Tupperware) - free!

---

## Optional Components

### Data Logging & Display

| Item | Quantity | Unit Price (£) | Supplier | Purpose |
|------|----------|----------------|----------|---------|
| SD Card Module | 1 | 4.99 | [Amazon UK](https://www.amazon.co.uk/AZDelivery-Reader-Module-Arduino-including/dp/B077MB17JB) | Standalone data logging |
| microSD Card (8GB) | 1 | 4.49 | Amazon UK | For SD module |
| 16×2 LCD Display (I2C) | 1 | 5.99 | [Amazon UK](https://www.amazon.co.uk/AZDelivery-Display-Arduino-Raspberry-including/dp/B07D83DY17) | Real-time temp display |
| 0.96" OLED Display | 1 | 6.99 | Amazon UK | Graphical display option |

**Benefits:**
- No PC needed for data collection
- Real-time monitoring without Serial
- Portable/embedded operation

---

### Testing & Debug

| Item | Quantity | Unit Price (£) | Supplier | Purpose |
|------|----------|----------------|----------|---------|
| 5V Fan (40mm) | 1 | 4.99 | [Amazon UK](https://www.amazon.co.uk/40mm-Cooling-Brushless-40x40x10mm-5V/dp/B07PXQCBVY) | Disturbance testing |
| Ice Cubes / Cold Pack | - | Free | Kitchen | Disturbance testing |
| Infrared Thermometer | 1 | 12.99 | Amazon UK | Verify sensor accuracy |
| USB Power Bank | 1 | Already owned | - | Portable Arduino power |

---

## Tools Required

### Essential (Assumed Available)

| Tool | Purpose | Notes |
|------|---------|-------|
| Computer with USB | Arduino programming | Mac/Windows/Linux |
| USB A-B Cable | Arduino connection | Usually included with Arduino |
| Digital Multimeter | Testing, debugging | Already owned |

### Recommended (Optional)

| Tool | Price | Purpose |
|------|-------|---------|
| Soldering Iron | £15-30 | Permanent connections |
| Wire Strippers | £8 | Clean wire prep |
| Heat Shrink Tubing | £5 | Insulate connections |
| Hot Glue Gun | £10 | Secure components |

---

## Cost Summary

### Minimum Viable Build

| Category | Cost (£) |
|----------|----------|
| Temperature Sensor (DS18B20) | 6.99 |
| Solid State Relay (SSR-25DA) | 5.99 |
| Heating Pad (5W) | 8.99 |
| Power Supply (12V 2A) | 7.99 |
| Container | 1.00 |
| **SUBTOTAL (New Purchases)** | **30.96** |
| Components Already Owned | 0.00 |
| **TOTAL PROJECT COST** | **£30.96** |

---

### Enhanced Build (with Optional Components)

| Additional Item | Cost (£) |
|-----------------|----------|
| SD Card Module + Card | 9.48 |
| LCD Display (16×2 I2C) | 5.99 |
| 5V Fan (disturbance testing) | 4.99 |
| Insulation Material | 2.00 |
| **TOTAL ENHANCED BUILD** | **£53.42** |

---

## Purchasing Strategy

### Option 1: All from Amazon UK (Fast)
- **Delivery:** 1-2 days (Prime) or 3-5 days (free)
- **Advantages:** Easy returns, reliable
- **Total:** ~£31

### Option 2: Mix of Suppliers (Budget)
- **Amazon UK:** Quick items (sensor, SSR)
- **AliExpress:** Slow items (heating pad, power supply)
- **Advantages:** 30-50% cheaper
- **Disadvantages:** 2-3 weeks delivery
- **Total:** ~£18-22

### Option 3: Local Electronics Store (Immediate)
- **RS Components** (rs-online.com) - Collection available
- **CPC Farnell** (cpc.farnell.com)
- **Maplin** (if still open locally)
- **Advantages:** Same-day pickup
- **Disadvantages:** Higher prices, limited stock
- **Total:** ~£40-50

**Recommendation:** Amazon UK for convenience and reasonable pricing

---

## Supplier Quick Links

### UK Suppliers

**Amazon UK** (General components)
- https://www.amazon.co.uk

**RS Components** (Professional, quality guaranteed)
- https://uk.rs-online.com

**CPC Farnell** (Educational discounts available)
- https://cpc.farnell.com

**Pimoroni** (Raspberry Pi/Arduino specialists)
- https://shop.pimoroni.com

**The Pi Hut** (Maker-friendly)
- https://thepihut.com

### International (Cheaper, Slower)

**AliExpress** (2-3 weeks delivery)
- https://www.aliexpress.com

**Banggood** (2-3 weeks delivery)
- https://www.banggood.com

---

## Safety & Compliance Notes

### Electrical Safety

⚠️ **Important Safety Considerations:**

1. **Mains Voltage:** This project uses **low voltage DC only** (12V). The SSR is rated for mains, but we're using it with 12V DC for safety.

2. **Thermal Safety:**
   - Heating pad max temp: 100°C
   - Water boiling point: 100°C at sea level
   - **Recommendation:** Set software limit to 80°C max

3. **Water + Electronics:**
   - Keep Arduino and SSR away from water
   - Use waterproof sensor only
   - No mains voltage near water

4. **Fire Safety:**
   - Never leave heating system unattended
   - Use non-flammable container
   - Ensure adequate ventilation

### Component Ratings

| Component | Voltage Rating | Current Rating | Thermal Rating | Safety Margin |
|-----------|---------------|----------------|----------------|---------------|
| DS18B20 | 5.5V max | 1.5mA | 125°C max | Good |
| SSR-25DA | 380VAC | 25A | 80°C operating | Excellent |
| Heating Pad | 12VDC | 0.42A | 100°C max | Good |
| Arduino Uno | 5V logic | 500mA (pins) | 70°C | Adequate |

---

## Version History

**v1.0** - December 2025 - Initial BOM  
**Last Updated:** December 2025

---

## Notes

- All prices in GBP (£) as of December 2025
- Prices may vary; check current listings
- Some components may be available cheaper in multi-packs
- Consider buying 2x temperature sensors for redundancy/backup

---

**Return to:** [Main README](../README.md) | [Hardware Documentation](wiring_diagram.md)