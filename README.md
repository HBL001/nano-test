# AeroHalo — Embedded Firmware (Arduino/PlatformIO)

Firmware for the **AeroHalo** module, built with **PlatformIO** and developed in **VS Code**. It drives a high‑side P‑MOSFET (via an NPN gate driver) from an **Arduino Nano** to control LED/fan loads with clean PWM.

---

## Repository Contents

- **`platformio.ini`** — PlatformIO project configuration (board, build flags, monitor settings).
- **`src/`** — Production firmware sources.
- **`test/`** — Hardware test sketches (e.g., PWM dimmer cycle).
- **`.vscode/`** and supporting files — Editor/tasks/debug settings for VS Code + PlatformIO.

> The **`main`** branch tracks a **tested/stable** build. Create feature branches for changes, then PR into `main`.

---

## Quick Start

### Using VS Code + PlatformIO
1. Open the folder in **VS Code**.
2. Install the **PlatformIO** extension (if not already installed).
3. In the PlatformIO sidebar:
   - Select the environment for **Nano (ATmega328P)**.
   - **Build** → **Upload** → **Monitor**.

### Using the CLI
```bash
pio run                 # build
pio run -t upload       # flash
pio device monitor      # serial monitor
```

---

## Hardware Reference (AeroHalo Driver)

- **MCU:** Arduino Nano (ATmega328P), PWM on **D9**.
- **Driver:** **MMBT3904** NPN pulls the P‑MOSFET **gate** low.
- **Switch:** **IPP80P03P4L** P‑MOSFET as a **high‑side** switch.
- **Typical values:**
  - D9 → **10 kΩ** → NPN **base** (use **3.3–4.7 kΩ** if raising PWM into multi‑kHz).
  - Gate **pull‑up** (Gate→Source): **22 kΩ** (to MOSFET **Source**, i.e., the +V LED rail).
  - Gate **series** (NPN collector→Gate): **47–100 Ω** (edge damping).
  - **Common ground** between Nano, driver, and load supply.

