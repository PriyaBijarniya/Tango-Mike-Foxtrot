# ⚠️ Safety Guidelines for TMF Development

The Tango-Mike-Foxtrot platform uses **high-voltage electric fields, plasma discharges, and high-frequency pulsed electromagnetic accelerators**. These are not standard hobbyist-grade components. If you are not trained in handling high-voltage equipment or inertial field effects, **DO NOT proceed without supervision or proper equipment**.

This file outlines **critical safety considerations** for anyone attempting to build, test, or modify the TMF prototype.

---

## ⚡ High Voltage: Ion Shell Systems

- Plasma field emitters operate between **12,000 – 30,000 volts DC**.
- Exposure to corona fields may cause **serious or fatal electrical shock**.
- Ensure **Faraday cage**–style shielding around test zones when energized.
- Use **gloves rated to 35kV**, and keep one hand behind your back when working near live systems.
- Disable and discharge all capacitors before touching emitter rings.

**💥 Safety Tip:** Use *interlock relays* connected to the ground line so the system disables automatically if a panel is opened or a tool is inserted.

---

## 🧲 Pulsed Electromagnetic Fields

- TMF’s pulse system relies on **supercapacitor-fed EM coils** switching at kilohertz to megahertz intervals.
- These can induce **eddy currents**, **burns**, or **neurological interference** if improperly shielded.
- Never power EM cores near:
  - **Pacemakers or neurostim devices**
  - **Hard drives or magnetic sensors**
  - **Unshielded laptops, phones, or AGI processors**

**⚠️ Note:** Some TMF coil arrays may induce **micro-vibrations** in metal tools or nearby components. This is normal — but dangerous near conductive surfaces or if left running unattended.

---

## 🧪 Inertial Oscillation Systems

TMF may use **alternating capacitor-discharge inertial field drivers**. These systems:

- Can cause **sudden vibration**, shake, or brief horizontal motion of the drone frame
- May **resonate at frequencies that stress lightweight mounts or 3D-printed plastic**
- Should always be **clamped down during initial bench tests**
- Should **never be tested indoors unless on a reinforced vibration table**

---

## 🔥 Heat Generation (Plasma Layer)

- Ion emitters create **thermal envelopes** exceeding 600–1000°C at peak
- Hull should be protected with:
  - **Kapton thermal tape**
  - **Ceramic-layered composite panels**
  - **Internal ceramic shielding behind the emitters**

Use **thermal imaging** or temp-sensitive paint to monitor hull temperatures during stress tests.

---

## 🧯 Fire & Arc Risk

- Keep **Class C fire extinguishers** nearby during test runs
- Install **fused breakers** inline with every HV stage
- Arc discharges may occur between emitter rings and nearby wiring — ensure **spatial separation and HV insulation shielding**

---

## 📋 Final Precautions

| Risk Type        | Safety Equipment |
|------------------|------------------|
| High Voltage     | 35kV-rated gloves, grounding stick |
| EM Coil Drivers  | EM shielding, non-metal table, metal tool distance |
| Inertial Motion  | Test clamp-down straps, reinforced test bench |
| Plasma Field     | Kapton tape, ceramic shielding, temp sensors |
| Fire/Arc         | Class C extinguisher, arc barrier gaps |

---

## 🔒 Legal Notice

By attempting to build or replicate the TMF system, **you accept all personal risk**. This system **does not conform to FAA, FCC, or CE regulations**, and is **experimental**. You may not test it in public airspace or near humans without proper authorization.

---

> This isn’t RC. This is aerospace engineering with your name on the patent or the tombstone. Treat it like you’re building something that could pierce the veil.
