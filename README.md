# ibooster-can-research-kit
CAN research + bench-control experiments for Bosch iBooster (notes, logs, code, and references). This is a “one place” archive of what I learned and what sources helped. No warranty. Experimental. Do not use on public roads.



# iBooster CAN Compendium (Experimental, Bench-Only)

**This repository is a community-style “one place” collection of code + notes I used to research and bench-control a Bosch iBooster (Gen2) over CAN.**  
It’s meant to save future builders time by consolidating working patterns, tooling, and references.

> **SAFETY / LEGAL DISCLAIMER (READ FIRST)**
>
> - **EXPERIMENTAL RESEARCH ONLY. BENCH USE ONLY.**
> - **NOT safety-certified. NOT validated for public-road use.**
> - Braking is safety-critical. A mistake can cause injury, death, or property damage.
> - **NO WARRANTY. NO LIABILITY.** You assume all risk for use, misuse, or modification.
> - Do not use this repo to operate a vehicle on public roads.
> - Respect laws, regulations, and responsible disclosure.

---

## What this repo contains

### ✅ Arduino / Embedded bring-up (primary)
- **Dual MCP2515 setup** (Arduino Mega2560 + 2 CAN modules) targeting iBooster’s **dual CAN**:
  - **YAW CAN**: receive **0x39D** (rod position / status) and decode to human-readable values
  - **VEH CAN**: transmit **0x38D / 0x38B / 0x38C** control-related frames for bench experiments
- **Non-blocking scheduling** (no long `delay()` loops)
- **Serial CLI** for enabling/disabling streams and setting simple request values
- **Pluggable checksum/CRC** experimentation (ex: CRC8 J1850 vs SUM) with clear TODOs

Location: `arduino/`

### ✅ Tools (optional, but helpful)
- Scripts for parsing logs and/or generating playback patterns for CAN tools  
Location: `tools/`

### ✅ Docs
- Wiring guidance, termination basics, CAN tool setup notes, troubleshooting  
Location: `docs/`

---

## Quick start (bench setup)

### Hardware assumptions
- Arduino Mega2560
- 2× MCP2515 modules (common boards)
- 500 kbps CAN
- MCP2515 crystal may be **8 MHz** or **16 MHz** (set this in config)

### Wiring (typical)
- **YAW MCP2515**: CS=4, INT=3 (configurable)
- **VEH MCP2515**: CS=5, INT=2 (configurable)
- Make sure CANH/CANL are correct and you have proper termination **on the bench network**.

> iBooster channels are often **not terminated internally**. Your test network typically needs **one 120Ω at each end** of the bus.

---

## Repository layout

- `arduino/`  
  Arduino sketches + helper headers for frame packing/unpacking and checksum experiments.
- `tools/`  
  Optional helper scripts for log parsing / playback generation.
- `docs/`  
  Wiring, SavvyCAN notes, troubleshooting, bench safety checklist.
- `ACKNOWLEDGEMENTS.md`  
  Credit and links to all public sources that informed this work.
- `DISCLAIMER.md`  
  Full disclaimer text (repeat of the warnings above, but more explicit).
- `LICENSE`  
  Open-source license (copyleft recommended if you want changes shared back).

---

## What this repo is *not*
- Not an OEM DBC.
- Not a complete reverse-engineering of every signal.
- Not a production brake controller.
- Not a “plug-and-play” automotive braking solution.

This is **research + experimentation**, written to be readable and verifiable.

---

## Attribution / Credits (how this repo handles it)

This project is built from **many public sources** (forums, open repos, documentation, tool authors, and community discoveries). I’m intentionally collecting what helped me in one place — **with credit**.

### Where credits live
- **`ACKNOWLEDGEMENTS.md`**: full list of sources/tools/people + what they contributed
- Inline comments in code for specific logic borrowed/adapted (frame IDs, checksum hypotheses, decoding hints, etc.)

### If you contributed ideas or code
Open an issue/PR and I’ll add you to acknowledgements. If anything is missing or attributed incorrectly, please tell me — I want this to be respectful and accurate.

---

## Contributing (please read)
- Keep contributions bench-focused and clearly labeled experimental
- Don’t post proprietary documents, leaked OEM material, or anything copyrighted
- Prefer small, testable PRs (one improvement per PR)
- Add notes: what was tested, on what hardware, and what the result was

---

## License
This repo uses a copyleft license so improvements remain open.  
See `LICENSE` for details.

---

## Contact / Support
Use GitHub Issues for:
- “No motion” troubleshooting (include logs, wiring, CAN bitrate, crystal freq)
- Checksum/CRC validation notes
- New signal discoveries (include evidence and how you tested)

---

## Final warning (seriously)
Brakes can hurt people. Don’t use experimental code on public roads.  
Bench-test with safe setups (current limiting, fusing, physical restraints, E-stop, etc.).
