Both datasets named in the deck are public. The organizers' degradation script is not.

## EuRoC MAV
- **Official page (ETH Zurich ASL):** <https://projects.asl.ethz.ch/datasets/euroc-mav/> The page lists it as the 2016 EuRoC MAV Dataset, with DOI 10.3929/ethz-b-000690084.
- **Which format to get:** The ORB-SLAM3 docs say to download the ASL-format zips, not the ROS bags. The ASL zips contain `mav0/imu0/data.csv`, which matches the deck's input format.
- **Sequences:** 11 in total (MH_01 to MH_05, V1_01 to V1_03, V2_01 to V2_03). For tuning you need the IMU file and the ground truth, which is under `mav0/state_groundtruth_estimate0/`.
- **Legacy URLs:** Older projects fetch the ROS bags from `robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/<sequence>/`. The zips may sit in the same folders, but the official page above is the safer starting point.

## TUM-VI
- **Official page (TUM CVG):** <https://cvg.cit.tum.de/data/datasets/vi-dataset>
- **Format and size:** The page offers EuRoC-format tars at 1024×1024 and 512×512 resolution, and corridor1 is 9.75 GB at 1024 versus 3.28 GB at 512. Take the **512×512** versions, since you only need the IMU and ground truth.
- **Batch download:** The page gives a wget recipe that mirrors `https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/`.
- **Ground truth caveat:** The IMU runs at 200 Hz, but the motion-capture ground truth covers only the start and end of each sequence. From my own knowledge, only the `room1`–`room6` sequences have ground truth for the whole trajectory, so use those for full-trajectory scoring. Please check this on the page.

## Not public
- **Degradation script and student kit:** the deck gives no link, and the final submission link opens only to registered teams on 8 Nov. Check your registration email, and ask prism@samsung.com if nothing arrives.
- **Hidden ground truth:** the evaluators keep it, so tune on EuRoC and TUM-VI with our own surrogate degradation script until the official one arrives.

## Practical tips
- **Bandwidth:** the zips and tars include camera images and are large. If the server supports range requests, the Python `remotezip` package can pull just `imu0/data.csv` and the ground-truth CSV out of a remote zip. I haven't tested this against these servers.
- **`evo` ground truth:** `evo` can read EuRoC ground-truth CSVs, and `evo_traj` can convert them to TUM format with `--save_as_tum`. Confirm with `--help` on your installed version.
- **Licensing:** check the terms on each dataset page, and keep the data out of your GitHub repo (the PRD already requires this).
- **Input path:** the deck says `imu0/data.csv`, but the public EuRoC layout is `mav0/imu0/data.csv`. Make the parser accept both.

I can add a "Data setup" section to the PRD with these links, a download script and the parser change.