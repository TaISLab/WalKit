

## Calibration tools (`scripts/`)

Used to find where the lidar hits the walker's legs, so those points can be filtered out.

- `find_minimums.py`: prints, for every scan, the indices whose range is below a threshold.
  `ros2 run walker_laser_filter find_minimums.py`
- `laser_filter_generator.py`: accumulates scans for a while and writes a `LaserScanMaskFilter`
  parameter file (`demofile3.txt`, in the current directory) with the indices that are below the
  threshold with at least a given probability.
  `ros2 run walker_laser_filter laser_filter_generator.py`

Thresholds and topic are hardcoded in each script's `main()`.
