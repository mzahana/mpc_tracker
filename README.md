# mpc_tracker
An implementation of a linear Model Predictive Controller (MPC) for optimal trajectory generation

# Installation
Execute the `setup.sh` script in the scripts directory.
```bash
cd scripts
./setup.sh
```

# Testing
* Adjust cnfiguration parameters in `config/mpc_tracker.yaml`

* The following launch file runs the `mpc_tracker_node`, with a test case.
    ```bash
    roslaunch mpc_tracker mpc_tracker.launch
    ```
