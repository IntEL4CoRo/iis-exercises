# Integrated Intelligent Systems Course Exercises

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/IntEL4CoRo/iis-exercises.git/dev)

Exercises of EDX course [Integrated Intelligent Systems](https://edx.intel4coro.de/courses/course-v1:UNI_BREMEN+IIS+2024_T2/about).

## Run Image Locally

- Run Docker image

  ```bash
  docker compose up -d --build 
  ```

- Open url http://localhost:8888/

## Enable nvidia GPU and display GUI applications on host machine

To display GUI applications on your host machine instead of a virtual display.
Uncomment the following configs in [docker-compose.yml](./docker-compose.yml)

```docker-compose
    #   - /tmp/.X11-unix:/tmp/.X11-unix:rw
    # environment:
    #   - DISPLAY
    #   - NVIDIA_DRIVER_CAPABILITIES=all
    # deploy:
    #   resources:
    #     reservations:
    #       devices:
    #         - driver: nvidia
    #           count: all
    #           capabilities: [gpu]
```

and run `docker compose up` with X-forwarding:

```bash
xhost +local:docker && \
docker compose up && \
xhost -local:docker
```

## Troubleshooting

- JupyterLab instance crashed when running `colcon build`.

  Solution: Limit the number of building threads: `colcon build --parallel-workers 2`

## License

Copyright 2023 IntEL4CoRo\<intel4coro@uni-bremen.de\>

This repository is released under the Apache License 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License 2.0.

### Acknowledgements

This Docker image is based on [jupyter/docker-stacks](https://github.com/jupyter/docker-stacks), licensed under the [BSD License](https://github.com/jupyter/docker-stacks/blob/main/LICENSE.md).

Gazebo example referneces [Tiryoh/docker-ros2-desktop-vnc](https://github.com/Tiryoh/docker-ros2-desktop-vnc), licensed under the [Apache License 2.0](https://github.com/Tiryoh/docker-ros2-desktop-vnc/blob/master/LICENSE).
