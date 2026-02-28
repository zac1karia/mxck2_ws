# MXCarkit – Technische Dokumentation

> Umfassende technische Referenz für das MXCarkit ROS 2 Humble Workspace (`mxck2_ws`).

---

## Inhaltsverzeichnis

1. [Projektübersicht & Zweck](#1-projektübersicht--zweck)
2. [Hardware-Architektur](#2-hardware-architektur)
3. [Software-Struktur](#3-software-struktur)
4. [Fahrzeug-Steuerkette (Vehicle Control Pipeline)](#4-fahrzeug-steuerkette-vehicle-control-pipeline)
5. [Core-Module & Funktionalität](#5-core-module--funktionalität)
6. [Docker & Containerisierung](#6-docker--containerisierung)
7. [ROS 2 Integration](#7-ros-2-integration)
8. [Testing, Quality Assurance & Best Practices](#8-testing-quality-assurance--best-practices)

---

## 1. Projektübersicht & Zweck

### Was ist MXCarkit?

MXCarkit ist eine modulare, ROS 2-basierte Roboterfahrzeug-Plattform, die auf einem ferngesteuerten (RC) Modellfahrzeug aufbaut. Das Projekt kombiniert Echtzeit-Sensorik, GPU-beschleunigtes Computing und eine vollständig containerisierte Software-Architektur, um autonome Fahrfunktionen zu ermöglichen.

### Hauptziele und Use-Cases

| Ziel | Beschreibung |
|------|-------------|
| **Fernsteuerung** | Manuelle Fahrzeugkontrolle über RC-Fernbedienung oder Gamepad |
| **Autonomes Fahren** | Wahrnehmungs- und Entscheidungspipelines für selbstständiges Fahren |
| **Sensor-Fusion** | Integration von LiDAR, Kamera (RGB-D), IMU und Ultraschall |
| **Lehre & Forschung** | Lernplattform für ROS 2, Robotik und Computer Vision |
| **Rapid Prototyping** | Modularer Aufbau für schnelle Entwicklung und Tests neuer Algorithmen |

### Entwicklungsstatus und Reife

- **ROS-Distribution**: ROS 2 Humble Hawksbill (LTS)
- **Betriebssystem**: Ubuntu 22.04 (Jammy) / Ubuntu L4T (Linux for Tegra)
- **Status**: Aktive Entwicklung mit stabilem Core-Stack (`mxck2_ws`) und separatem GPU-Workspace (`development_ws`)
- **Versionierung**: Zwei Packages auf Version `0.0.0` – frühe, aber funktionale Phase

---

## 2. Hardware-Architektur

### Systemübersicht

```
┌──────────────────────────────────────────────────────────────────────┐
│                        MXCarkit Hardware                            │
│                                                                      │
│  ┌─────────────┐    USB Hub    ┌──────────────────┐                 │
│  │  Powerbank  │──────────────▶│  NVIDIA Jetson   │                 │
│  │  (5V USB)   │               │  (JetPack SDK)   │                 │
│  └─────────────┘               └──────┬───────────┘                 │
│                                       │                              │
│          ┌────────────────────────────┼────────────────────┐        │
│          │              │             │           │        │        │
│          ▼              ▼             ▼           ▼        ▼        │
│   ┌────────────┐ ┌───────────┐ ┌──────────┐ ┌────────┐ ┌────┐    │
│   │ STM32      │ │ RPLidar   │ │ Intel    │ │  VESC  │ │USB │    │
│   │ Nucleo     │ │ A2M12     │ │ D435i    │ │        │ │Hub │    │
│   │ (micro-USB)│ │ (USB)     │ │ (USB)    │ │ (USB)  │ │    │    │
│   └─────┬──────┘ └───────────┘ └──────────┘ └───┬────┘ └────┘    │
│         │                                        │                  │
│    ┌────┴────┐                            ┌──────┴──────┐          │
│    │Sensoren │                            │  Aktoren    │          │
│    ├─────────┤                            ├─────────────┤          │
│    │RC Recv. │                            │ BLDC Motor  │          │
│    │USS      │                            │ Servo Motor │          │
│    │IMU      │                            └──────┬──────┘          │
│    │LED Drvr.│                                   │                  │
│    └─────────┘                            ┌──────┴──────┐          │
│                                           │ 4S LiPo     │          │
│                                           │ Battery      │          │
│                                           └─────────────┘          │
└──────────────────────────────────────────────────────────────────────┘
```

### NVIDIA Jetson (Computing Unit)

- **Rolle**: Zentraler Rechenknoten für ROS 2, Sensorverarbeitung und KI-Inferenz
- **OS**: Ubuntu L4T (Linux for Tegra) mit JetPack SDK
- **Stromversorgung**: USB-Powerbank (geteilt mit USB Hub)
- **Power-Modi**: 15W / 20W Varianten mit unterschiedlicher Kernanzahl (konfigurierbar via `nvpmodel`)
- **Überwachung**: `tegrastats`, `jtop` für GPU/CPU/Memory-Monitoring

### STM32 Nucleo Mikrocontroller

- **Verbindung**: Micro-USB → Jetson (`/dev/stm32_nucleo`)
- **Kommunikation**: micro-ROS (ROS 2) mit serieller Baudrate **921600**
- **Aufgaben**:
  - Empfängt PWM-Signale vom RC-Receiver
  - Liest IMU-Daten (Beschleunigung, Gyroskop)
  - Liest Ultraschall-Sensordaten
  - Sendet ROS-Messages an den Jetson
  - Empfängt Befehle zur LED-Steuerung (bidirektional)
- **Veröffentlichte Topics**:
  - `/veh_remote_ctrl` (Int16MultiArray – PWM-Werte)
  - `/imu` (IMU-Daten)
  - `/uss_sensors` (Ultraschall-Sensordaten)

### VESC (Vedder Electronic Speed Controller)

- **Verbindung**: USB → Jetson (`/dev/vesc`)
- **Firmware**: VESC-Firmware (spezifisch für das Fahrzeug)
- **Steuert**: BLDC-Motor (Antrieb) und Servo-Motor (Lenkung)
- **Stromversorgung**: 4S LiPo Batterie

**VESC-Konfigurationswerte** (`vesc_config.yaml`):

| Parameter | Wert |
|-----------|------|
| `port` | `/dev/vesc` |
| `speed_max` | 23250.0 ERPM |
| `speed_min` | -23250.0 ERPM |
| `servo_max` | 0.9 |
| `servo_min` | 0.1 |
| `current_max` | 100.0 A |
| `brake_max` | 200000.0 |
| `brake_min` | -20000.0 |

### Sensor-Integration

#### RPLidar A2M12
- **Typ**: 2D-Laserscanner
- **Verbindung**: USB → Jetson
- **Topic**: `/scan`
- **Launch**: `rplidar_a2m12_launch.py`

#### Intel D435i RealSense
- **Typ**: RGB-D Kamera mit integriertem IMU
- **Verbindung**: USB → Jetson
- **Modi**:

| Modus | Profil | Beschreibung |
|-------|--------|-------------|
| RGB | 640×360 @ 15 FPS, BGR8 | Farbkamera |
| Depth | 640×480 @ 15 FPS | Tiefensensor |
| RGB-D | 640×360 @ 15 FPS | Farbe + Tiefe kombiniert |
| IMU | Accel + Gyro | Inertialsensor |
| IR Links/Rechts | 640×360 @ 15 FPS | Infrarot-Kameras |
| IR Projector | On/Off | Infrarot-Emitter |

- **Topics**: `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`, `/camera/camera/imu`, etc.

#### IMU (über STM32 Nucleo)
- **Topic**: `/imu`
- **Daten**: Beschleunigung + Gyroskop

#### Ultraschall-Sensoren
- **Topic**: `/uss_sensors`
- **Daten**: Abstandsmessungen

### Schnittstellen und Kommunikationsprotokolle

| Gerät | Schnittstelle | Protokoll | Baudrate/Details |
|-------|--------------|-----------|-----------------|
| STM32 Nucleo | USB (serial) | micro-ROS | 921600 Baud |
| VESC | USB (serial) | VESC-Protokoll | `/dev/vesc` |
| RPLidar | USB | Slamtec-Protokoll | – |
| Intel D435i | USB 3.0 | UVC / HID | – |
| RC Receiver | PWM → STM32 | PWM (1000–2000 µs) | – |

### Stromversorgung

| Komponente | Quelle | Details |
|------------|--------|---------|
| Jetson + USB Hub | Powerbank (USB) | 5V-Versorgung |
| VESC + Motoren | 4S LiPo Batterie | 14.8V nominell |

---

## 3. Software-Struktur

### ROS 2 Humble Workspace Architektur

```
mxck2_ws/                          ← Haupt-Workspace
├── src/
│   ├── mxck_run/                  ← Zentrales Launch- und Utility-Package
│   │   ├── launch/
│   │   │   ├── mxck_run_launch.py        ← Master-Launchfile
│   │   │   ├── realsense_launch.py       ← Kamera-spezifisch
│   │   │   ├── broadcast_tf_launch.py    ← TF-Publisher
│   │   │   └── record_launch.py          ← Bag-Recording
│   │   ├── mxck_run/
│   │   │   └── message_utils.py          ← ROS-Message-Hilfsfunktionen
│   │   ├── test/
│   │   │   ├── test_flake8.py
│   │   │   ├── test_pep257.py
│   │   │   └── test_copyright.py
│   │   └── urdf/
│   │       └── mxcarkit.urdf             ← Robotermodell
│   │
│   └── vehicle_control/           ← Fahrzeug-Steuerketten-Package
│       ├── launch/
│       │   └── manual_control_launch.py
│       ├── config/
│       │   ├── control_config.yaml       ← Steuerungsparameter
│       │   └── vesc_config.yaml          ← VESC-Parameter
│       └── vehicle_control/
│           ├── rc_to_joy.py              ← RC → Joy
│           ├── joy_to_ackermann.py       ← Joy → Ackermann
│           └── ackermann_to_vesc.py      ← Ackermann → VESC
│
├── Dockerfile                     ← Produktions-Image
├── Dockerfile.ubuntu              ← Base-Image (ROS Humble)
├── Dockerfile.l4t                 ← Base-Image (Jetson/GPU)
├── ros_entrypoint.sh              ← Workspace-Overlay-Skript
├── .devcontainer/
│   ├── docker-compose.yml         ← Einfache Konfiguration
│   └── docker-compose.all.yml    ← Alle Services
└── .bash_aliases                  ← Convenience-Befehle
```

### Zwei separate ROS Workspaces

| Workspace | Zweck | Base Image | GPU |
|-----------|-------|------------|-----|
| `mxck2_ws` | Core-Funktionalität: Sensorik, Steuerung, Launch-Management | `ros:humble-ros-base-jammy` | Nein |
| `development_ws` | Fahrfunktionen mit GPU-Beschleunigung (Neuronale Netze, Perception) | `ultralytics/ultralytics:latest-jetson-jetpack5` | Ja |

### Packages

#### `mxck_run`
- **Typ**: ament_python
- **Rolle**: Zentraler Startpunkt – enthält keine eigenen ROS-Nodes, sondern Launch-Files zum Orchestrieren aller Sensoren und Aktoren
- **Dependencies**: rclpy, std_msgs, sensor_msgs, ackermann_msgs, geometry_msgs, nav_msgs, cv_bridge, transforms3d
- **Enthält**: `message_utils.py` als Library für Message-Erstellung

#### `vehicle_control`
- **Typ**: ament_python
- **Rolle**: Fahrzeug-Steuerkette (3 Nodes)
- **Dependencies**: rclpy, std_msgs, sensor_msgs, ackermann_msgs
- **Entry Points**:
  ```python
  'console_scripts': [
      'rc_to_joy = vehicle_control.rc_to_joy:main',
      'joy_to_ackermann = vehicle_control.joy_to_ackermann:main',
      'ackermann_to_vesc = vehicle_control.ackermann_to_vesc:main',
  ]
  ```

### Docker-Containerisierung und Base Images

Die Software läuft vollständig in Docker-Containern. Kein Software wird direkt auf dem Jetson installiert. Siehe [Abschnitt 6](#6-docker--containerisierung) für Details.

---

## 4. Fahrzeug-Steuerkette (Vehicle Control Pipeline)

### Übersicht

```
RC-Fernbedienung                Gamepad/Joystick
      │                               │
      ▼                               ▼
┌─────────────┐               ┌─────────────┐
│ RC Receiver │               │  joy_node   │
│ (PWM Signal)│               │ (ROS 2 pkg) │
└──────┬──────┘               └──────┬──────┘
       │                              │
       ▼                              │
┌─────────────┐                       │
│ STM32 Nucleo│                       │
│ (micro-ROS) │                       │
└──────┬──────┘                       │
       │                              │
       ▼                              │
  /veh_remote_ctrl                    │
  (Int16MultiArray)                   │
       │                              │
       ▼                              │
┌──────────────┐                      │
│  rc_to_joy   │                      │
│  (Node)      │                      │
└──────┬───────┘                      │
       │                              │
       ▼                              ▼
     /rc/joy ◄──────────────────── /joy
     (Joy msg)                  (Joy msg)
       │                           │
       └──────────┬────────────────┘
                  ▼
         ┌─────────────────┐
         │ joy_to_ackermann│
         │     (Node)      │
         └────────┬────────┘
                  │
                  ▼
          /rc/ackermann_cmd
       (AckermannDriveStamped)
                  │
                  ▼
         ┌─────────────────┐
         │ackermann_to_vesc│
         │     (Node)      │
         └────────┬────────┘
                  │
        ┌─────────┼──────────┐
        ▼         ▼          ▼
  /commands/  /commands/  /commands/
  motor/      servo/      motor/
  speed       position    brake
  (Float64)   (Float64)   (Float64)
        │         │          │
        └────┬────┘──────────┘
             ▼
      ┌─────────────┐
      │    VESC     │
      │  Controller │
      └──────┬──────┘
             │
        ┌────┴────┐
        ▼         ▼
   BLDC Motor  Servo Motor
   (Antrieb)   (Lenkung)
```

### Schritt 1: RC Signal → Joy Message (`rc_to_joy.py`)

**Node-Name**: `rc_to_joy`

Der `RCJoystick`-Node konvertiert rohe PWM-Signale vom RC-Receiver (empfangen über den STM32 Nucleo) in standardisierte `sensor_msgs/Joy`-Messages.

**Eingabe**: `/veh_remote_ctrl` (`Int16MultiArray` – 3 PWM-Werte: Steering, Speed, Mode)

**Ausgabe**: `/rc/joy` (`sensor_msgs/Joy`)

**PWM-Kalibrierung** (aus `control_config.yaml`):

| Kanal | Min PWM | Mid PWM | Max PWM | Mapping |
|-------|---------|---------|---------|---------|
| Steering | 1000 | 1500 | 2000 | -1.0 → 0.0 → +1.0 |
| Speed | 1000 | 1500 | 2000 | -1.0 → 0.0 → +1.0 |
| Mode | 1000 | 1500 | 2000 | Index 0, 1 oder 2 |

**Kernlogik** – PWM zu normalisiertem Wert:

```python
def get_interp(x_vals, y_vals):
    return lambda x: np.interp(x, x_vals, y_vals)

# Mapping: PWM → normalisierter Joy-Wert (-1.0 bis +1.0)
self.steer_mapping = get_interp(
    (steer_min_pwm, steer_mid_pwm, steer_max_pwm),
    (-1.0, 0.0, 1.0)
)
```

**Mode-Erkennung** – Der nächste kalibrierte PWM-Wert wird per `argmin` ausgewählt:

```python
mode_val = np.argmin(np.abs(self.mode_values - mode_pwm))
# Ergibt 0 (Deadman), 1 (Autonomous) oder 2 (Manual)
```

**Verbindungsverlust-Erkennung**: Wenn ein PWM-Wert 0 ist, werden alle Ausgaben auf Null gesetzt.

### Schritt 2: Joy Message → Ackermann Drive (`joy_to_ackermann.py`)

**Node-Name**: `joy_control`

Der `JoyControl`-Node konvertiert Joy-Messages (von RC oder Gamepad) in `AckermannDriveStamped`-Messages.

**Eingabe**: `/rc/joy` (RC-Modus) oder `/joy` (Joystick-Modus)

**Ausgabe**: `/rc/ackermann_cmd` (`AckermannDriveStamped`)

**Konfigurationswerte**:

| Parameter | Wert | Beschreibung |
|-----------|------|-------------|
| `steering_angle_max` | 0.44 rad (~25.2°) | Maximaler Lenkwinkel |
| `max_forward_speed` | 2.0 m/s | Maximale Vorwärtsgeschwindigkeit |
| `max_backward_speed` | -2.0 m/s | Maximale Rückwärtsgeschwindigkeit |
| `joy_deadzone` | 0.07 | Totzone des Joysticks |
| `erpm_min` | 700 | Minimale ERPM-Schwelle |
| `speed_to_erpm_gain` | 3786 | ERPM = Gain × Speed (m/s) |

**Deadzone-Mapping** – Stellt sicher, dass kleine Joystick-Auslenkungen ignoriert werden:

```python
# Lenkung: Deadzone → 0, danach linear skaliert
self.steer_mapping = get_interp(
    (-1.0, -joy_deadzone, joy_deadzone, 1.0),
    (-steer_max, 0.0, 0.0, steer_max)
)

# Geschwindigkeit: Deadzone → 0, darüber/darunter Mindestgeschwindigkeit
speed_min = erpm_min / speed_to_erpm_gain  # ≈ 0.185 m/s

self.speed_mapping = get_interp(
    (-1.0, -joy_deadzone, -joy_deadzone+ε, joy_deadzone-ε, joy_deadzone, 1.0),
    (max_backward_speed, -speed_min, 0.0, 0.0, speed_min, max_forward_speed)
)
```

**Achsen-Zuordnung** (konfigurierbar):

| Control Type | Steering Axis | Speed Axis |
|-------------|--------------|-----------|
| RC | 0 | 1 |
| Joystick | 3 | 4 |

### Schritt 3: Ackermann Drive → VESC Befehle (`ackermann_to_vesc.py`)

**Node-Name**: `ackermann_to_vesc`

Der `AckermannToVesc`-Node ist der zentrale Steuerknoten. Er konvertiert `AckermannDriveStamped`-Messages in VESC-Motor- und Servo-Befehle und implementiert dabei Sicherheitslogik.

**Eingabe**:
- `/rc/ackermann_cmd` (manuell) oder `/autonomous/ackermann_cmd` (autonom)
- `/rc/joy` oder `/joy` (für Mode-Updates)

**Ausgabe**:
- `/commands/motor/speed` (Float64 – ERPM)
- `/commands/servo/position` (Float64 – Wert 0.0 bis 1.0)
- `/commands/motor/brake` (Float64 – Bremsstrom in Ampere)

**Umrechnungsformeln**:

```python
# ERPM-Berechnung
erpm = speed_to_erpm_gain * speed  # 3786 × speed (m/s)

# Servo-Position-Berechnung
servo_value = servo_mid + steering_sign * steering_angle * steer_to_servo_gain
# Bsp: 0.443 + (-1) × 0.44 × 1.0 = 0.003 (Links-Einschlag)

# Clamping auf gültige Werte
servo_value = max(min(servo_value, servo_max), servo_min)
# Ergebnis: max(min(0.003, 0.9), 0.1) = 0.1
```

**VESC Servo-Werte**:

| Parameter | Wert | Bedeutung |
|-----------|------|-----------|
| `servo_min` | 0.1 | Maximaler Links-Einschlag (mit Sicherheitspuffer) |
| `servo_mid` | 0.443 | Neutralstellung (Geradeaus) |
| `servo_max` | 0.9 | Maximaler Rechts-Einschlag (mit Sicherheitspuffer) |
| `steer_to_servo_gain` | 1.0 | Umrechnungsfaktor Lenkwinkel → Servo |
| `brake_amps` | -20.0 A | Bremsstrom |
| `invert_steering` | true | Lenkung invertiert (Fahrzeug-spezifisch) |

---

## 5. Core-Module & Funktionalität

### Parameter Management System

Parameter werden in YAML-Dateien definiert und über das ROS 2 Parameter-System geladen:

```
vehicle_control/config/
├── control_config.yaml    ← Steuerungsparameter (für alle 3 Nodes)
└── vesc_config.yaml       ← VESC-Hardware-Parameter
```

**Dynamisches Parameter-Reload**: Der `ackermann_to_vesc`-Node lädt Parameter automatisch alle 10 Sekunden neu:

```python
# Timer für periodisches Neuladen
self.timer = self.create_timer(10.0, self.load_params)
```

Dies ermöglicht das Ändern von Parametern zur Laufzeit (z.B. `servo_mid` justieren), ohne den Node neuzustarten.

**Namespaced Parameters** – Jeder Node hat eigene Parameter unter seinem Namen:

```yaml
/**:               # Gilt für alle Nodes
  ros__parameters:
    control_type: "rc"
    erpm_min: 700
    speed_to_erpm_gain: 3786

rc_to_joy:         # Nur für rc_to_joy
  ros__parameters:
    steering_min_pwm: 1000

joy_to_ackermann:  # Nur für joy_to_ackermann
  ros__parameters:
    steering_angle_max: 0.44

ackermann_to_vesc: # Nur für ackermann_to_vesc
  ros__parameters:
    servo_mid: 0.443
```

### Mode Management (Deadman, Manual, Autonomous)

Der `ackermann_to_vesc`-Node implementiert drei Fahrmodi:

| Modus | RC-Wert | Joy-Werte | Beschreibung |
|-------|---------|-----------|-------------|
| **Deadman** | 0 | btn[4]=0, btn[5]=0 | Keine Fahrt – Sicherheitsmodus |
| **Autonomous** | 1 | btn[4]=0, btn[5]=1 | Autonome Steuerung aktiv |
| **Manual** | 2 | btn[4]=1, btn[5]=0 oder 1 | Manuelle Steuerung aktiv |

**Joystick Mode-Matrix** (2×2):

```python
#                    auto_btn=0    auto_btn=1
# mode_btn=0  →    [0 (Deadman),  2 (Autonomous)]
# mode_btn=1  →    [1 (Manual),   1 (Manual)]
self.mode_matrix = [[0, 2],
                    [1, 1]]
```

**Modus-Wechsel-Logik**: Bei jedem Moduswechsel wird automatisch eine Notbremsung ausgeführt:

```python
def update_mode(self, msg):
    new_mode = msg.buttons[self.mode_btn]  # RC
    if new_mode != self.mode:
        self.brake()  # Notbremsung bei Moduswechsel
        self.mode = new_mode
```

### Safety Check Mechanismus (8-Sekunden-Kalibrierung)

Beim Start des `ackermann_to_vesc`-Nodes muss eine Sicherheitsprüfung bestanden werden, bevor Fahrbefehle akzeptiert werden:

**Ablauf**:

1. Node startet im **Kalibrierungsmodus** – Fahr-Subscriber sind noch nicht aktiv
2. Benutzer muss den Deadman-Modus aktivieren
3. System sammelt Geschwindigkeitswerte über **8 Sekunden** bei **40 Hz** = **320 Werte**
4. Alle 320 Werte müssen exakt **0.0 m/s** sein (Gas und Lenkung nicht berühren)
5. Nach erfolgreicher Kalibrierung: akustisches Servo-Signal und Aktivierung der Fahr-Subscriber

```python
# Safety-Check-Parameter
n_seconds = 8    # Dauer der Kalibrierung
hz = 40          # Erwartete Frequenz
self.min_values = n_seconds * hz  # = 320 Mindestwerte

def safety_check(self, ackermann_msg):
    if self.mode != self.dead_val:
        return
    speed = ackermann_msg.drive.speed
    self.speed_values.append(speed)
    if len(self.speed_values) > self.min_values:
        self.speed_values.pop(0)
        if max(self.speed_values) == 0 and min(self.speed_values) == 0:
            self.get_logger().info("Calibration complete!")
            self.destroy_subscription(self.safety_sub)
            self.signal_calibration_complete()
            self.initialize_subscribers()
```

**Kalibierungs-Signal**: Nach erfolgreicher Kalibrierung bewegt sich das Servo in einer Sinuswelle als visuelles Feedback:

```python
def signal_calibration_complete(self):
    amplitude = 0.2
    frequency = 3.0
    vertical_shift = 0.5
    t = np.linspace(0, np.pi, 60)
    values = amplitude * np.sin(frequency * t) + vertical_shift
    # Servo bewegt sich 60 Schritte bei 40 Hz ≈ 1.5 Sekunden
```

### Emergency Brake Logik

Notbremsungen werden in zwei Situationen ausgelöst:

1. **Moduswechsel**: Jeder Wechsel zwischen Deadman/Manual/Autonomous löst eine Bremsung aus
2. **Geschwindigkeit erreicht 0**: Wenn die Zielgeschwindigkeit auf 0 fällt (und vorher ≠ 0 war)

```python
def brake(self):
    hz = 420
    for _ in range(hz):
        self.brake_pub.publish(self.brake_msg)  # -20.0 A
        time.sleep(1/hz)  # ≈ 1 Sekunde intensive Bremsung
```

Zusätzlich wird gebremst, wenn die berechnete ERPM unter dem Minimum liegt:

```python
if abs(erpm) < self.erpm_min:  # < 700 ERPM
    self.brake_pub.publish(self.brake_msg)
else:
    self.erpm_pub.publish(self.erpm_msg)
```

### ROS Topic Struktur

#### Steuerketten-Topics

| Topic | Typ | Richtung | Beschreibung |
|-------|-----|----------|-------------|
| `/veh_remote_ctrl` | `Int16MultiArray` | STM32 → Jetson | PWM-Rohwerte (Steering, Speed, Mode) |
| `/rc/joy` | `Joy` | rc_to_joy → joy_to_ack | Normalisierte RC-Joystick-Werte |
| `/joy` | `Joy` | joy_node → joy_to_ack | Gamepad-Joystick-Werte |
| `/rc/ackermann_cmd` | `AckermannDriveStamped` | joy_to_ack → ack_to_vesc | Manuelle Fahrbefehle |
| `/autonomous/ackermann_cmd` | `AckermannDriveStamped` | Autonomie → ack_to_vesc | Autonome Fahrbefehle |
| `/commands/motor/speed` | `Float64` | ack_to_vesc → VESC | Motor-ERPM |
| `/commands/servo/position` | `Float64` | ack_to_vesc → VESC | Servo-Position (0.0–1.0) |
| `/commands/motor/brake` | `Float64` | ack_to_vesc → VESC | Bremsstrom (-20.0 A) |

#### Sensor-Topics

| Topic | Typ | Quelle |
|-------|-----|--------|
| `/scan` | `LaserScan` | RPLidar |
| `/camera/camera/color/image_raw` | `Image` | RealSense RGB |
| `/camera/camera/depth/image_rect_raw` | `Image` | RealSense Depth |
| `/camera/camera/imu` | `Imu` | RealSense IMU |
| `/camera/camera/infra1/image_rect_raw` | `Image` | IR links |
| `/camera/camera/infra2/image_rect_raw` | `Image` | IR rechts |
| `/camera/camera/rgbd` | `RGBD` | RGB-D Composit |
| `/imu` | `Imu` | STM32 IMU |
| `/uss_sensors` | – | Ultraschall |

#### System-Topics

| Topic | Typ | Beschreibung |
|-------|-----|-------------|
| `/tf_static` | `TFMessage` | Statische Transformationen (URDF) |
| `/robot_description` | `String` | URDF-Modell |

---

## 6. Docker & Containerisierung

### Base Images

Das Projekt verwendet drei Dockerfile-Stufen:

#### 1. `Dockerfile.ubuntu` – Humble Base Image (ohne GPU)

```dockerfile
FROM ros:humble-ros-base-jammy
```

**Enthält**:
- RealSense2 SDK + ROS-Packages
- micro-ROS Agent Workspace (`/microros_ws`)
- VESC Driver Workspace (`/vesc_ws`)
- RPLIDAR ROS2 Package (`/rplidar_ws`)
- Foxglove Bridge, Joy, Robot State Publisher
- setuptools 58.2.0, transforms3d, ultralytics

**Verwendung**: Sensorik, Steuerung, Foxglove – alle Nicht-GPU-Aufgaben

#### 2. `Dockerfile.l4t` – Jetson/GPU Base Image

```dockerfile
FROM ultralytics/ultralytics:latest-jetson-jetpack5
```

**Enthält**:
- ROS 2 Foxy (auf Jetson wegen CUDA-Kompatibilität)
- librealsense (aus Source kompiliert)
- micro-ROS, VESC, RPLIDAR Workspaces
- vision_msgs (humble Branch für neuere Message-Typen)
- PyCUDA, TensorRT-Anbindung
- NumPy 1.23 (vermeidet PyCUDA/TensorRT-Konflikte)

**Verwendung**: GPU-beschleunigte Perception und KI-Inferenz

#### 3. `Dockerfile` – Produktions-Image

```dockerfile
FROM mxwilliam/mxck:mxck-humble-ubuntu-22.04
```

**Fügt hinzu**:
- `ros2_numpy` Library
- `ros_entrypoint.sh` (Workspace-Overlay)
- `.bash_aliases` (Convenience-Befehle)

### Workspace Overlaying

Das `ros_entrypoint.sh`-Skript sourced alle Workspaces in korrekter Reihenfolge:

```
1. /opt/ros/humble/setup.bash        ← ROS 2 Basis
2. /microros_ws/install/setup.bash    ← micro-ROS Agent
3. /vesc_ws/install/setup.bash        ← VESC Driver
4. /rplidar_ws/install/setup.bash     ← RPLIDAR
5. /mxck2_ws/install/setup.bash       ← MXCarkit (wird bei Bedarf gebaut)
```

Falls der mxck2_ws noch nicht gebaut wurde, führt das Skript automatisch `colcon build --symlink-install` aus.

### Portainer Integration

- **Port**: 9000 (automatisch bei Boot gestartet)
- **URL**: `http://<ip-address>:9000`
- **Funktion**: Web-basierte Container-Verwaltung
- **Prinzip**: Jeder Container hat nur eine Aufgabe ("one container, one concern")

### Container Orchestrierung

**docker-compose.yml** (Basis):

```yaml
services:
  control:
    container_name: mxck2_control
    privileged: true
    network_mode: mxck0021-net
    volumes:
      - /home/mxck/mxck2_ws:/mxck2_ws
      - /dev:/dev
```

**docker-compose.all.yml** (Alle Services):

| Service | Container | Funktion |
|---------|-----------|----------|
| `control` | `mxck2_control` | Basis-Shell |
| `foxglove` | `mxck2_foxglove` | Visualisierung (Port 8765) |
| `camera` | `mxck2_camera` | RGB-Kamera |
| `micro` | `mxck2_micro` | micro-ROS Agent |
| `kickstart` | `mxck2_kickstart` | Manuelle Fahrzeugsteuerung |
| `lidar` | `mxck2_lidar` | LiDAR-Scanner |

**Gemeinsame Einstellungen**:
- `privileged: true` – Zugriff auf Hardware-Geräte
- `network_mode: mxck0021-net` – Gemeinsames Docker-Netzwerk
- Volume-Mounts: `/mxck2_ws` (Code) und `/dev` (Geräte)

---

## 7. ROS 2 Integration

### Topic Whitelist und Publishing

Der Foxglove Bridge wird mit einer expliziten Topic-Whitelist konfiguriert, die definiert, welche Topics an das Visualisierungstool weitergeleitet werden:

```python
TOPIC_WHITELIST = [
    # Base Topics
    "/tf_static", "/rc/ackermann_cmd", "/autonomous/ackermann_cmd", "/scan",
    # Micro-ROS Topics
    "/imu", "/uss_sensors", "/veh_remote_ctrl",
    # Camera Topics
    "/camera/camera/color/image_raw", "/camera/color/image_jpeg",
    "/camera/camera/imu", "/camera/camera/depth/image_rect_raw",
    "/camera/camera/infra1/image_rect_raw", "/camera/camera/infra2/image_rect_raw",
    "/camera/camera/depth/color/points", "/camera/camera/rgbd",
    # Custom Topics
    "/pdc", "/position", "/path", "/result", "/waypoint",
    "/detections_2d", "/detections_3d",
]
```

> **Hinweis**: Foxglove zeigt `/tf_static` standardmäßig nicht an – daher ist die Whitelist-Konfiguration wichtig.

### QoS Profile

Alle Steuerketten-Nodes verwenden das `qos_profile_sensor_data` Profil mit Tiefe 1:

```python
from rclpy.qos import qos_profile_sensor_data

qos_profile = qos_profile_sensor_data
qos_profile.depth = 1  # Nur neueste Nachricht behalten
```

**Bedeutung**:
- **Reliability**: Best Effort (kein Re-Transmit)
- **Durability**: Volatile (keine History für neue Subscriber)
- **History Depth**: 1 (nur die letzte Nachricht)
- **Ideal für**: Echtzeit-Sensorik und Steuerung, wo Aktualität wichtiger als Vollständigkeit ist

### Launch Files und deren Argumente

#### `mxck_run_launch.py` – Master-Launchfile

| Argument | Default | Beschreibung |
|----------|---------|-------------|
| `run_foxglove` | `false` | Startet Foxglove Bridge (Port 8765) |
| `run_camera` | `false` | Startet RGB-Kamera |
| `run_lidar` | `false` | Startet RPLidar A2M12 |
| `run_micro` | `false` | Startet micro-ROS Agent |
| `broadcast_tf` | `false` | Publiziert TF-Frames (URDF) |
| `run_motors` | `false` | Startet manuelle Fahrzeugsteuerung |
| `run_rs_imu` | `false` | Aktiviert RealSense IMU |

**Beispielaufruf**:
```bash
ros2 launch mxck_run mxck_run_launch.py run_camera:=true run_lidar:=true broadcast_tf:=true
```

#### `realsense_launch.py` – Kamera-Launch

| Argument | Default | Beschreibung |
|----------|---------|-------------|
| `camera` | `false` | RGB-Kamera |
| `rgbd` | `false` | RGB-D Composit |
| `rs_imu` | `false` | Accelerometer + Gyroscope |
| `ir_left` | `false` | Linke IR-Kamera |
| `ir_right` | `false` | Rechte IR-Kamera |
| `ir_projector` | `false` | IR-Emitter |

**IR-Projector-Steuerung**: Der IR-Emitter wird nach 3 Sekunden über eine `TimerAction` aktiviert, um sicherzustellen, dass der Kamera-Node bereit ist:

```python
TimerAction(
    period=3.0,
    actions=[OpaqueFunction(function=set_emitter_param)]
)
```

#### `manual_control_launch.py` – Manuelle Steuerung

Startet die vollständige Steuerkette basierend auf `control_type`:

**RC-Modus** (`control_type: "rc"`):
- `vesc_driver_node` + `micro_ros_agent` + `rc_to_joy` + `joy_to_ackermann` + `ackermann_to_vesc`

**Joystick-Modus** (`control_type: "joy"`):
- `vesc_driver_node` + `joy_node` + `joy_to_ackermann` + `ackermann_to_vesc`

#### `record_launch.py` – Bag-Recording

| Argument | Default | Beschreibung |
|----------|---------|-------------|
| `filename` | Zufällig (z.B. `calm_dolphin_42`) | Name der Bag-Datei |
| `format` | `mcap` | Speicherformat (`mcap` oder `sqlite3`) |

**Aufgezeichnete Topics**: `/rc/ackermann_cmd`, `/camera/imu`, `/imu`

#### `broadcast_tf_launch.py` – TF-Publisher

- Lädt URDF aus dem Package-Share-Verzeichnis
- Startet `robot_state_publisher` mit 20 Hz Publish-Frequenz
- Publiziert `/tf_static` einmalig (alle Joints sind fixed)

### Message Utilities (`message_utils.py`)

Die `message_utils.py`-Library bietet Hilfsfunktionen für die Erstellung von ROS 2 Messages:

| Funktion | Beschreibung |
|----------|-------------|
| `create_ackermann_msg(speed, angle)` | Erstellt `AckermannDriveStamped` |
| `create_compressed_image_message(cv_image)` | NumPy-Bild → `CompressedImage` (JPEG) |
| `create_compressed_grayscale_image_message(cv_image)` | Grayscale → `CompressedImage` |
| `create_ros_image(numpy_image)` | NumPy → `sensor_msgs/Image` (via cv_bridge) |
| `create_pose_message(point, angle)` | Position + Yaw → `PoseStamped` |
| `create_path_message(waypoints)` | Waypoints [x,y,θ] → `nav_msgs/Path` |
| `create_point_cloud_message(points)` | 2D/3D-Punkte → `PointCloud2` |
| `get_relative_transform(source, target)` | TF-Lookup mit Retry-Logik (max 5s) |
| `angle_to_quaternion(angle)` | Yaw (rad) → `Quaternion` |
| `image_msg_to_numpy(image_msg)` | `Image` → NumPy-Array |
| `compressed_image_msg_to_numpy(compressed_msg)` | `CompressedImage` → NumPy-Array |

---

## 8. Testing, Quality Assurance & Best Practices

### PEP8, PEP257, Copyright Tests

Das `mxck_run`-Package enthält drei automatisierte Qualitätstests:

#### Flake8 (PEP8) – Code-Style

```python
# test/test_flake8.py
@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
```

#### PEP257 – Docstring-Konventionen

```python
# test/test_pep257.py
@pytest.mark.linter
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
```

#### Copyright – Header-Prüfung

```python
# test/test_copyright.py
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
```

> **Hinweis**: Der Copyright-Test ist derzeit übersprungen (`@pytest.mark.skip`).

### Node Management Patterns

#### Duplikat-Erkennung via Launch File

Das empfohlene Muster prüft vor dem Start, ob ein Node bereits läuft:

```python
# In der Launch-Datei
result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
running_nodes = [n.lstrip('/') for n in result.stdout.strip().split('\n')] if result.stdout else []

for name, node in node_names.items():
    if name not in running_nodes:
        ld.add_action(node)
```

**Vorteile**:
- Zentrale Kontrolle über alle Nodes
- Funktioniert auch mit externen/unveränderbaren Nodes (z.B. Kameratreiber)
- Node-Namen können in Launch Files überschrieben werden

### Respawn und Crash Handling

Kritische Nodes (VESC Driver, micro-ROS Agent) sind mit Respawn konfiguriert:

```python
vesc = Node(
    package='vesc_driver',
    executable='vesc_driver_node',
    name='vesc_driver_node',
    parameters=[vesc_config],
    respawn=True,         # Automatischer Neustart bei Crash
    respawn_delay=20.0    # 20 Sekunden Verzögerung vor Neustart
)
```

**Respawn-konfigurierte Nodes**:

| Node | Respawn Delay | Grund |
|------|--------------|-------|
| `vesc_driver_node` | 20s | Hardware-Treiber – muss immer laufen |
| `micro_ros_agent` | 20s | Kommunikationsbrücke zum STM32 |

### Convenience Aliases

Für schnelle Bedienung stehen Shell-Aliases bereit (`.bash_aliases`):

| Alias | Befehl | Funktion |
|-------|--------|----------|
| `kickstart` | `ros2 launch vehicle_control manual_control_launch.py` | RC-Steuerung starten |
| `run_foxglove` | `mxck_run_launch.py run_foxglove:=true broadcast_tf:=true` | Visualisierung |
| `run_camera` | `realsense_launch.py camera:=true` | RGB-Kamera |
| `run_rgbd` | `realsense_launch.py camera:=true rgbd:=true` | RGB-D Modus |
| `run_imu` | `realsense_launch.py rs_imu:=true` | IMU-Sensor |
| `run_lidar` | `mxck_run_launch.py run_lidar:=true` | LiDAR-Scanner |
| `run_micro` | `mxck_run_launch.py run_micro:=true` | micro-ROS Agent |
| `run_vio` | `realsense_launch.py camera:=true rs_imu:=true` | Visual-Inertial Odometry |
