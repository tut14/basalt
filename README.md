# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) improved
for tracking XR devices with
[Monado](https://gitlab.freedesktop.org/monado/monado). Many thanks to the
Basalt authors.

## Installation

- **Prebuilt (Ubuntu/Raspberry/Radxa)**: Download [latest .deb](https://gitlab.freedesktop.org/mateosss/basalt/-/releases) and install with

  ```bash
  sudo apt install -y ./basalt-monado-*.deb
  ```

- **From source (Linux)**

  ```bash
  git clone --recursive https://gitlab.freedesktop.org/mateosss/basalt.git
  cd basalt && ./scripts/install_deps.sh
  cmake --preset library # use "development" instead of "library" if you want extra binaries and debug symbols
  sudo cmake --build build --target install
  ```

- **From source (Windows)**: See the [build guide](doc/monado/Windows.md) for Windows.

## Usage

If you want to run OpenXR application with Monado, you need to set the
environment variable `VIT_SYSTEM_LIBRARY_PATH` to the path of the basalt library.

By default, Monado will try to load the library from `/usr/lib/libbasalt.so` if
the environment variable is not set.

If you want to test whether everything is working you can download a short dataset with [EuRoC (ASL) format](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format like [`MOO09_short_1_updown`](https://huggingface.co/datasets/collabora/monado-slam-datasets/resolve/main/M_monado_datasets/MO_odyssey_plus/MOO_others/MOO09_short_1_updown.zip?download=true) from the [Monado SLAM datasets](https://huggingface.co/datasets/collabora/monado-slam-datasets):

```bash
wget https://huggingface.co/datasets/collabora/monado-slam-datasets/resolve/main/M_monado_datasets/MO_odyssey_plus/MOO_others/MOO09_short_1_updown.zip
unzip MOO09_short_1_updown.zip
```

- **Try it standalone with a dataset (requires extra binaries)**

  ```bash
  basalt_vio --show-gui 1 --dataset-path MOO09_short_1_updown/ --dataset-type euroc --cam-calib /usr/share/basalt/msdmo_calib.json --config-path /usr/share/basalt/msdmo_config.json
  ```

- **Use a RealSense camera without Monado (requires extra binaries)**
  You'll need to calibrate your camera if you want the best results but meanwhile you can try with these calibration files instead.

  - RealSense D455 (and maybe also D435)

    ```bash
    basalt_rs_t265_vio --is-d455 --cam-calib /usr/share/basalt/d455_calib.json --config-path /usr/share/basalt/default_config.json
    ```

  - Realsense T265: Get t265_calib.json from [this issue](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) and run

    ```bash
    basalt_rs_t265_vio --cam-calib t265_calib.json --config-path /usr/share/basalt/default_config.json
    ```

- **Try it through `monado-cli` with a dataset**

  ```bash
  monado-cli slambatch MOO09_short_1_updown/ /usr/share/basalt/msdmo.toml results
  ```

- **Try it with `monado`, a dataset, and an OpenXR app**

  ```bash
  # Run monado-service with a fake "euroc device" driver
  export EUROC_PATH=MOO09_short_1_updown/ # dataset path
  export EUROC_HMD=false # false for controller tracking
  export EUROC_PLAY_FROM_START=true # produce samples right away
  export SLAM_CONFIG=/usr/share/basalt/msdmo.toml # includes calibration
  export SLAM_SUBMIT_FROM_START=true # consume samples right away
  export XRT_DEBUG_GUI=1 # enable monado debug ui
  monado-service &

  # Get and run a sample OpenXR application
  wget https://gitlab.freedesktop.org/wallbraker/apps/-/raw/main/VirtualGround-x86_64.AppImage
  chmod +x VirtualGround-x86_64.AppImage
  ./VirtualGround-x86_64.AppImage normal
  ```

- **Use a real device in Monado**.

  When using a real device driver you might want to enable the `XRT_DEBUG_GUI=1` and `SLAM_UI=1` environment variables to show debug GUIs of Monado and Basalt respectively.

  Monado has a couple of drivers supporting SLAM tracking (and thus Basalt). Most of them should work without any user input.

  - WMR ([troubleshoot](doc/monado/WMR.md))
  - Rift S (might need to press "Submit to SLAM", like the Vive Driver).
  - Northstar / DepthAI ([This hand-tracking guide](https://monado.freedesktop.org/handtracking) has a depthai section).
  - Vive Driver (Valve Index) ([read before using](doc/monado/Vive.md))
  - RealSense Driver ([setup](doc/monado/Realsense.md)).

## Development

If you want to set up your build environment for developing and iterating on Basalt, see the [development guide](doc/Development.md).

# CI Improvements
Sometimes the build is cached from a different branch!!!
Datasets locations are hardcoded
You need to start the gitlab runner in $ssd home
You need to run for docker builds:
  - module load container/rootless-docker
  - export DOCKER_TMPDIR=$H/Desktop/docker-stuff/tmp
  - export DOCKER_DATAROOT=$H/Desktop/docker-stuff/
  - start_rootless_docker.sh
You need to have two instances, one docker (for arm crosscompilation) and one shell (for dataset evaluation)


# Command FFMPEG
ffmpeg -framerate 30 -pattern_type 'glob' -i 'data/*.png' -c:v libx264 -preset "fast" -tune "film" -x264opts "partitions=p8x8,p4x4,i8x8:keyint=1000:me=umh:merange=64:subme=6:bframes=0:ref=1" data.mp4



# Run commands




export run=mv0 usemv=0
export ds=TR3   dspath=dataset-room3_512_16 calib=data/tum/tumvi_512_ds_calib.json config=data/tum/tumvi_512_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=TR4   dspath=dataset-room4_512_16 calib=data/tum/tumvi_512_ds_calib.json config=data/tum/tumvi_512_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=TR6   dspath=dataset-room6_512_16 calib=data/tum/tumvi_512_ds_calib.json config=data/tum/tumvi_512_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MGO05 dspath=MGO05_inspect_easy   calib=data/msd/msdmg_calib.json        config=data/msd/msdmg_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MGO07 dspath=MGO07_mapping_easy   calib=data/msd/msdmg_calib.json        config=data/msd/msdmg_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EMH01 dspath=MH_01_easy           calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EMH04 dspath=MH_04_difficult      calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EMH05 dspath=MH_05_difficult      calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MIO07 dspath=MIO07_mapping_easy   calib=data/msd/msdmi_calib.json        config=data/msd/msdmi_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MOO05 dspath=MOO05_inspect_easy   calib=data/msd/msdmo_calib.json        config=data/msd/msdmo_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MOO07 dspath=MOO07_mapping_easy   calib=data/msd/msdmo_calib.json        config=data/msd/msdmo_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EV101 dspath=V1_01_easy           calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log

export run=mv1 usemv=1
export ds=TR3   dspath=dataset-room3_512_16 calib=data/tum/tumvi_512_ds_calib.json config=data/tum/tumvi_512_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=TR4   dspath=dataset-room4_512_16 calib=data/tum/tumvi_512_ds_calib.json config=data/tum/tumvi_512_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=TR6   dspath=dataset-room6_512_16 calib=data/tum/tumvi_512_ds_calib.json config=data/tum/tumvi_512_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MGO05 dspath=MGO05_inspect_easy   calib=data/msd/msdmg_calib.json        config=data/msd/msdmg_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MGO07 dspath=MGO07_mapping_easy   calib=data/msd/msdmg_calib.json        config=data/msd/msdmg_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EMH01 dspath=MH_01_easy           calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EMH04 dspath=MH_04_difficult      calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EMH05 dspath=MH_05_difficult      calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MIO07 dspath=MIO07_mapping_easy   calib=data/msd/msdmi_calib.json        config=data/msd/msdmi_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MOO05 dspath=MOO05_inspect_easy   calib=data/msd/msdmo_calib.json        config=data/msd/msdmo_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=MOO07 dspath=MOO07_mapping_easy   calib=data/msd/msdmo_calib.json        config=data/msd/msdmo_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
export ds=EV101 dspath=V1_01_easy           calib=data/euroc/euroc_ds_calib.json   config=data/euroc/euroc_config.json
echo ">>> [$run - $ds]" && time ./build/basalt_vio --show-gui 0 --deterministic 1 --use-imu 0 --use-mvs $usemv --dataset-path ds/$dspath  --dataset-type euroc --cam-calib $calib --config-path $config --save-trajectory euroc 2>&1 | tee output.log; mkdir -p results/$run/$ds/ && mv trajectory.csv results/$run/$ds/tracking.csv && mv output.log results/$run/$ds/output.log
