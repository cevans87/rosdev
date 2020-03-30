# rosdev
A CLI for common ROS1/ROS2 developer tasks.

Rosdev makes developing, building, deploying ROS/ROS2 code simple and fast. It makes heavy use of...
* lazy evaluation - to ensure workspace dependencies such as source code, docker images, and
  binaries are in place before dependant actions are executed.
* persistent memoization - to skip expensive operations that have already been executed in the past
  and remember CLI and workspace settings.
* data-flow - to achieve high concurrency.

Distinct workflows should only require a single command.

## WARNING
Rosdev is a work in progress and lacks documentation and testing. It also covers a large surface
area of ROS/ROS2 development workflows. Expect some features to be shoddy or broken.

## Installation
Dependencies
* Python 3.8+
* Linux/MacOS
* Docker CLI

Installation steps
1. Follow [docker install instructions.](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
    * [Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
    * [Debian](https://docs.docker.com/install/linux/docker-ce/debian/)
    * [MacOS](https://docs.docker.com/docker-for-mac/install/)
2. Follow [docker post-install instructions.](https://docs.docker.com/install/linux/linux-postinstall/)
3. Install rosdev.
    ```bash
    > git clone https://github.com/cevans87/rosdev
    > sudo python3 setup.py install
    ```
4. Enable command line completion.
    ```bash
    > echo eval "$(register-python-argcomplete rosdev)" >> ~/.profile
    ```
    Unlike many Python CLIs that use argcomplete, Rosdev evaluates only a small subset of the
    overall software. Tab-completion is fast.

## Supported targets
* **ROS/ROS2** - all supported ROS/ROS2 releases and latest of ROS2.
* **OS** - only Ubuntu (for now).
* **Architecture** - arm32v7, arm64v8, and amd64.

## Usage
### Create a workspace
Creating a workspace is exactly the same as in the [colcon tutorial.](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace)

### Add sources
Adding sources is exactly the same as in the [colcon tutorial.](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#add-some-sources)

### (Automatic) Set target architecture and ROS release
The `--architecture` and `--release` flags are sticky for each workspace. When we use the flags for
any rosdev command, the flag value is saved. Future invocations of rosdev commands in this
workspace will automatically take the most-recently-applied `--architecture` and `--release`.
```bash
cevans@strahd:~/ros2_ws$ rosdev shell --architecture arm32v7 --release eloquent
<snip potentially lots of first-time build output>
cevans@rosdev_ros2_ws_eloquent_arm32v7:~/ros2_ws$ ^D
cevans@strahd:~/ros2_ws$ rosdev shell
cevans@rosdev_ros2_ws_eloquent_arm32v7:~/ros2_ws$ ^D
cevans@strahd:~/ros2_ws$ rosdev shell --architecture amd64 --release latest
<snip potentially lots of first-time build output>
cevans@rosdev_ros2_ws_latest_amd64:~/ros2_ws$ ^D
cevans@strahd:~/ros2_ws$ rosdev shell
cevans@rosdev_ros2_ws_latest_amd64:~/ros2_ws$ ^D
```

### (Automatic) Set Up local backend
The local backend is a docker container backed by an offical ROS docker image. Rosdev ensures this
step happens automatically. Relevant flags are `--architecture` and `--release`.
```bash
# Optionally specify --architecture and --release
cevans@strahd:~/ros2_ws$ rosdev shell 
<snip lots of first-time build output>
```

### (Optional) Set up remote build backend
If this step is skipped, builds fall back to the local backend.

A build backend may significantly reduce your build times. This is especially true if your
development machine is weak, it is a Mac (Docker for Mac incurs significant virtualization
overhead), or you are cross-compiling. For example, on my Macbook Pro 2015, cross-compiling a
project to arm64v8 took ~10 minutes. With my backend set to an EC2 Graviton a1.xlarge instance,
building took ~30 seconds.

Steps
1. Follow the instructions in [backend setup](#optional-backend-setup).
2. Use the `--backend-builder-uri` and, if needed, `--backend-builder-identity-path`. Like
   `--architecture` and `--release`, these flags are sticky.
   ```bash
   cevans@strahd:~/ros2_ws$ rosdev build --backend-builder-uri ssh://[user[:password]@]ip[:port]
   ```

### (Optional) Set up a remote runner backend
If this step is skipped, runs fall back to the local backend.

It is often desirable or necessary to run remotely, such as on a Raspberry Pi.

Steps
1. Follow the instructions in [backend setup](#optional-backend-setup).
2. Use the `--backend-runner-uri` and, if needed, `--backend-runner-identity-path`. Like
   `--architecture` and `--release`, these flags are sticky.
   ```bash
   cevans@strahd:~/ros2_ws$ rosdev run --backend-runner-uri ssh://[user[:password]@]ip[:port]
   ```

### (Optional) Backend setup
The backend is required to run the same OS and architecture as your local backend. You can determine
the OS and architecture of your local backend as follows.
```bash
cevans@strahd:~/ros2_ws$ rosdev shell --architecture arm64v8 --release eloquent
cevans@rosdev_ros2_ws_eloquent_arm64v8:~/ros2_ws$ lsb_release -d
Description:	Ubuntu 18.04.3 LTS
cevans@rosdev_ros2_ws_eloquent_arm64v8:~/ros2_ws$ uname -i
aarch64
```
In this case, Ubuntu 18.04.3 and aarch (arm64v8).

* Create a backend machine with the same OS and architecture, such as an EC2 Graviton instance or a
  Raspberry Pi.
* Ensure that you can ssh into the machine from your local host without a password prompt.
    * (recommended) For EC2, download the associated identity `.pem` file...
         * (recommended) into your local `~/.ssh` folder.
         * elsewhere on your filesystem. You'll specify the location of the file to rosdev with the
           `--backend-<backend_type>-identity-path` flag.
    * (recommended) For other machines such as a Raspberry Pi, it means enabling ssh-server on the
      backend, creating a ssh key pair on your local host, and uploading your public key ssh.
      ```bash
      cevans@strahd:~/ros2_ws$ ssh-keygen
      cevans@strahd:~/ros2_ws$ ssh-copy-id remote_user@10.0.0.50:22
      cevans@strahd:~/ros2_ws$ rosdev build --backend-builder-uri ssh://remote_user@10.0.0.50:22
      ```
    * (discouraged) Alternatively, if you use password authentication, you may provide the password
      in the backend URI.
      ```bash
      cevans@strahd:~/ros2_ws$ rosdev run \
        --backend-runner-uri ssh://cevans:my_runner_password@10.0.0.51:22
      ```

### Installing dependencies
Dependencies need to be installed in almost exactly the same way as with `rosdep install`. Rosdev
ensures that dependencies on the builder and runner backends are also installed, even if you add the
backends after installing dependencies on the local backend.
```bash
cevans@strahd:~/ros2_ws$ rosdev install "--from-paths src --ignore-src -r -y"
```

### Building
Building is almost exactly the same as with `colcon build`. It will build on your builder backend
(the local backend if a builder backend is not currently specified).
```bash
cevans@strahd:~/ros2_ws$ colcon build "--packages-up-to demo_nodes_cpp"
cevans@strahd:~/ros2_ws$ rosdev build --backend-builder-uri ssh://cevans@10.0.0.1 \
    "--packages-up-to demo_nodes_cpp"
```

### Running
Running is almost exactly the same as with `rosrun` or `ros2 run`. It will run on your runner
backend (the local backend if a runner backend is not currently specified).
```bash
cevans@strahd:~/ros2_ws$ rosdev run demo_nodes_cpp talker
cevans@strahd:~/ros2_ws$ rosdev run --backend-runner-uri ssh://cevans@10.0.0.1 \
    "demo_nodes_cpp talker"
```

### Launching
Launching is almost exactly the same as with `roslaunch` or `ros2 launch` (the local backend if a
builder backend is not currently specified).
```bash
cevans@strahd:~/ros2_ws$ rosdev launch demo_nodes_cpp talker_listener
cevans@strahd:~/ros2_ws$ rosdev launch --backend-runner-uri ssh://cevans@10.0.0.1 \
    demo_nodes_cpp talker_listener
```

## Running CLion or PyCharm

Once you've built your workspace at least once, you can start CLion or PyCharm with all the correct
settings for introspection, running code, debugging, and setting breakpoints. **Broken in CLion
2019.3 and later.**
```bash
cevans@strahd:~/ros2_ws$ rosdev build --architecture arm64v8 --release eloquent
cevans@strahd:~/ros2_ws$ rosdev clion
cevans@strahd:~/ros2_ws$ rosdev pycharm
```

### Resetting everything
If rosdev misbehaves, try using the `--reset-cache` flag. If that still doesn't work, delete the
`~/.rosdev` directory.
