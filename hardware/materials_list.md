# 🧰 Materials List — Tango-Mike-Foxtrot Prototype

A fully sourced bill of materials required to assemble the Tango-Mike-Foxtrot drone frame, plasma shell core, and power intake subsystems. All components are commercially available or fabricable on a civilian/hobbyist budget.

---

## 🧱 Frame + Structural

| Component             | Spec / Type                                 | Qty | Est. Cost | Source / Notes |
|----------------------|----------------------------------------------|-----|-----------|----------------|
| Carbon Fiber Tubing  | 22mm OD, 1.5mm wall, pre-preg, 1m length     | 6   | ~$120     | [dragonplate.com](https://dragonplate.com) |
| Kevlar Composite Sheet | 3mm panel, 18x24 in                         | 2   | ~$70      | For heat resistance under plasma shell |
| Aerospace Rivets     | Grade 8, 4mm head, titanium alloy            | 40  | ~$30      | Anti-corrosive fastening |
| Custom Machined Mounts | CNC'd aluminum or 3D printed PEKK          | 4   | ~$40      | Motor/coil alignment standoffs |

---

## ⚡ Power & Induction

| Component             | Spec / Type                                 | Qty | Est. Cost | Source / Notes |
|----------------------|----------------------------------------------|-----|-----------|----------------|
| Supercapacitor Bank  | 2.7V, 500F x 6 series                        | 1   | ~$90      | Maxwell or LS Mtron units |
| Graphene Ion Matrix  | 10-layer ultra-thin energy skin (DIY optional) | 1   | ~$150     | Also acts as heatsink layer |
| Induction Coil Array | Flat-wound copper, 12AWG, 8cm radius         | 4   | ~$60      | DIY or order custom wound |
| Oscillator Driver Board | ~900kHz-1.2MHz square pulse                | 2   | ~$35      | Prefab drone ESC alternative |

---

## 💠 Plasma Shell + Containment

| Component             | Spec / Type                                 | Qty | Est. Cost | Source / Notes |
|----------------------|----------------------------------------------|-----|-----------|----------------|
| Borosilicate Dome (half-sphere) | 6–8 in diameter, 3mm wall        | 1   | ~$20      | Lab glassware or repurposed lens |
| Plasma Igniter Node  | 20kV-35kV pulse, <30ms duty cycle            | 1   | ~$40      | HV module or xenon strobe circuit |
| Shell Stabilization Cage | Fine tungsten wire mesh                 | 1   | ~$25      | For grounding outer plasma field |
| Faraday Cage Layer   | 4x conductive mesh sheets, flexible          | 1   | ~$50      | Wraps core for EM containment |

---

## 🧠 Guidance & Feedback (Placeholder for /guidance_logic)

| Component             | Spec / Type                                 | Qty | Est. Cost | Source / Notes |
|----------------------|----------------------------------------------|-----|-----------|----------------|
| IMU Module           | 9-DOF, high-frequency MPU9250                | 1   | ~$12      | Standard drone use |
| Optical Flow Sensor  | PX4FLOW or equivalent                        | 1   | ~$25      | Redundant guidance |
| Thermocouples        | 1000°C rated x 2                             | 2   | ~$15      | Feedback loop to power control |
| Flight Control Board | F4 or F7 processor, custom firmware ready    | 1   | ~$40      | STM32 based |

---

## 🧩 Optional/Advanced Materials

| Component             | Why it's optional                            | Est. Cost | Notes |
|----------------------|-----------------------------------------------|-----------|-------|
| Aerogel Layer Sheet  | Optional for ultra-light insulation around coil zones | ~$80 | Harder to source but cool as hell |
| Monocrystalline Graphene Film | For ion channeling shell layers     | $100+     | Not required but improves discharge directionality |
| Heatsink Paste / Phase-Change Material | For power circuit cooling  | ~$10      | Prevents thermal overload in continuous test mode |

---

## 🧮 Estimated Total Cost

| Tier       | Est. Total |
|------------|-------------|
| Budget     | ~$420       |
| Mid        | ~$650       |
| Full Spec  | ~$920+      |

---  

## 🔗 Vendor Links (Non-Affiliate)
- [https://mouser.com](https://mouser.com)
- [https://dragonplate.com](https://dragonplate.com)
- [https://digikey.com](https://digikey.com)
- [https://aliexpress.com](https://aliexpress.com)
- [https://ebay.com](https://ebay.com) *(for HV modules, strobe boards, and wild-card parts)*

---

> ⚠️ **Warning:** Many components are safe, but *some high-voltage modules can be dangerous if misused*. Follow all electrical safety guidelines.

