# 🛠️ Assembly Instructions — TMF Prototype (Tango-Mike-Foxtrot)

This document outlines how to construct the physical airframe and core propulsion layout for the TMF drone. Assume civilian workshop conditions with basic tools (drill, saw, soldering station, safety gear). Estimated build time: 10–16 hours.

---

## 📦 1. Pre-Build Checklist

- [x] All materials from `/hardware/materials_list.md` sourced and verified
- [x] Workspace cleared (fire-resistant mat, grounded bench)
- [x] Safety gear (gloves, goggles, fume mask for soldering)
- [x] SVG frame diagram printed or referenced

---

## 🧱 2. Frame Construction

### 🧩 a. Carbon Frame Tubing
1. Cut carbon fiber tubes to:
   - 2x 100cm (main spars)
   - 2x 60cm (wing support)
   - 2x 40cm (rear stabilizers)
2. Use Kevlar composite panels to reinforce joints with epoxy bonding.
3. Insert and rivet aluminum or PEKK mounts at cross-points.

### 🧲 b. Coil Mounts
- Use CNC'd standoffs or 3D printed PEKK brackets.
- Secure one flat-wound coil on each wing tip and near rear, equidistant from plasma core.
- Leave ~1cm thermal gap around coils for airflow and wiring clearance.

---

## ⚡ 3. Power & Containment Layer

### 🔋 a. Install Supercapacitor Bank
- Mount in center cavity beneath core chamber.
- Use anti-vibration pads and shielded copper connections.
- Solder in parallel for redundancy.

### 🧬 b. Plasma Core Assembly
1. Mount borosilicate dome in center via ring clamp + shock mounts.
2. Fit ignition node at dome center, securing leads to frame ground mesh.
3. Wrap entire containment zone in Faraday mesh and bond to shared ground.

### 🧼 c. Faraday and Wire Routing
- All coil and igniter wires should route beneath the inner frame panel.
- Secure with heat-resistant sleeves and isolate HV lines from signal paths.

---

## 🔧 4. Control & Feedback Install

1. Mount IMU and optical flow modules on flat protected area away from EM fields.
2. Wire thermocouples into coil brackets and power driver units.
3. Attach flight board with custom firmware (see `/guidance_logic/` upcoming).
4. Do **not** connect to power until final signal check.

---

## 🚀 5. Final Checklist

- [ ] Frame passes twist/flex test (under 3mm deflection per wing tip)
- [ ] Plasma dome secured and aligned with coil geometry
- [ ] Power lines soldered, no cold joints
- [ ] Oscillator outputs test at 900kHz–1.2MHz (use oscilloscope)
- [ ] Control board boots and detects IMU + thermals

---

> ⚠️ **Note:** If you feel a *slight static charge* when placing hands near the plasma dome during power-up, this is **expected**. It confirms that the field is being generated.

---  

## 🏁 Ready for Initial Test

Your drone is now ready for logic flashing, static plasma test (no lift), and staged ramp-up. Do not arm full coil ignition without secured environment.

---

### 🔁 For SVG Schematic:
See [`/hardware/frame_specs.svg`](frame_specs.svg) and [`frame_specs.png`](frame_specs.png)
