# Robot-Aided-Drafter

The files hosted in this repository are used to implement the Robot Aided Drafter. 

[**Browse the full documentation »**]((https://charith-sunku.github.io/Robot-Aided-Drafter/))
---
<table>
<tr><td width="50%">

### Quick Features
* Code Repo for the RRP 3 DOF Robotic Plotter.  

</td><td>

<img src="robotarm_cad.png" width="320" alt="CAD render"/>

</td></tr></table>

---

## Repository Map

| Path | Contents |
|------|----------|
| **`Src/`** | Embedded C firmware (`main.c`, `motors.c`, etc.) |
| **`ImageToAngles.py`** | Raster → {θ₁, θ₂, Z} converter |
| **`verify_angles.mlx`** | MATLAB live script that re-plays joint lists |
| **`Doxyfile`** | Doxygen configuration (outputs to `docs/html`) |
| **`.github/workflows/doxygen.yml`** | CI pipeline & Pages deploy |
| **`docs/`** | _Auto-generated_ HTML documentation (ignored in Git) |

---

## How It Works

1. **Image Parsing**  
   `ImageToAngles.py` skeletonizes a black-&-white image, prunes
   waypoints (`--px-skip`, `--min-dist`), and runs inverse kinematics with an
   elbow offset of +147.5°.  
   Output → `image_thetas.txt`.

2. **Output Verification**  
   `verify_angles.mlx` performs forward kinematics and overlays the path on the original
   artwork—helpful for tuning spacing or resolution.

3. **Robot Motion**  
   Flash the STM32 firmware (`Src/`); it reads `plan[]` generated from the
   TXT file, homes, draws, then re-homes.  
   Safety is handled in real-time (limit switches + ultrasonic sensor).

---

## Building / Flashing

```bash
# prerequisites
sudo apt install gcc-arm-none-eabi stlink-tools make

cd Src
make             # builds *.elf and *.bin
st-flash write build/firmware.bin 0x08000000

