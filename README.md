# ida_data_logger — veri kayıt / data logging

**🇹🇷 [Türkçe](#türkçe) · 🇬🇧 [English](#english)**

---

## Türkçe

### Genel Bakış
Görev boyunca kanıt/telemetri kaydı üreten düğümler.
- **`video_logger`** — annotated MP4 video kaydı.
- **`lidar_video_logger`** — LiDAR görselleştirme videosu.
- **`telemetry_logger`** — GPS/hız/başlık telemetri CSV'si (≥1 Hz).
- **`costmap_logger`** — costmap ROS bag kaydı.

### Kurulum
> Önkoşullar: ROS 2 Humble, `colcon`, `rosdep`, Python + `pip`.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <REPO_URL> ida_data_logger   # ida_msgs'i de klonlayın
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/ida_data_logger/requirements.txt
colcon build --packages-select ida_data_logger
source install/setup.bash
```

### Kullanım
```bash
ros2 run ida_data_logger telemetry_logger --ros-args --params-file \
  src/ida_data_logger/config/logger_params.yaml
```

### Bağımlılıklar
ROS 2: `rclpy`, `sensor_msgs(_py)`, `nav_msgs`, `cv_bridge`, `rosbag2_py`,
`ida_msgs`. Pip: `numpy`, `opencv-python`.

### Lisans
**MIT.** Bulaşıcı bağımlılık yoktur.

**Kullanım koşulları:** Özgürce kullanın/değiştirin/dağıtın; lisans bildirimini
koruyun. Geliştirme yaparsanız bize **PR açmanız bizi mutlu eder** (zorunlu değil).

### Özel veri
Yoktur. Düğümler çalışma zamanında telemetri (lat/lon vb.) kaydeder; kodda gömülü
konum/geometri verisi bulunmaz. Üretilen kayıt dosyaları `.gitignore` ile hariç
tutulur.

---

## English

### Overview
Nodes that produce evidence/telemetry recordings during a mission.
- **`video_logger`** — annotated MP4 video.
- **`lidar_video_logger`** — LiDAR visualization video.
- **`telemetry_logger`** — GPS/speed/heading telemetry CSV (≥1 Hz).
- **`costmap_logger`** — costmap ROS bag.

### Installation
> Prerequisites: ROS 2 Humble, `colcon`, `rosdep`, Python + `pip`.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <REPO_URL> ida_data_logger   # also clone ida_msgs
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/ida_data_logger/requirements.txt
colcon build --packages-select ida_data_logger
source install/setup.bash
```

### Usage
```bash
ros2 run ida_data_logger telemetry_logger --ros-args --params-file \
  src/ida_data_logger/config/logger_params.yaml
```

### Dependencies
ROS 2: `rclpy`, `sensor_msgs(_py)`, `nav_msgs`, `cv_bridge`, `rosbag2_py`,
`ida_msgs`. Pip: `numpy`, `opencv-python`.

### License
**MIT.** No contagious dependency.

**Terms:** free to use/modify/distribute; preserve the license notice. If you
improve it, **a PR back to us would make us happy** (not required).

### Private data
None. The nodes record telemetry (lat/lon etc.) at runtime; no position/geometry
data is embedded in the code. Generated recordings are excluded via `.gitignore`.

---

<div align="center">

💙 **Bu Repo IEEE Ege Mavi İnci İnsansız Deniz Aracı Takımı Yazılım Ekibi Tarafından Oluşturulmuştur, Yazılım Ekibimize Sevgilerle**

[@NightKnight-nx2](https://github.com/NightKnight-nx2) · [@yalinoner](https://github.com/yalinoner) · [@nilayyldz](https://github.com/nilayyldz)

</div>
