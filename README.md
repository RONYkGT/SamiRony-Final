Clone this repo inside your `ros_ws` and build the dockerfile using:
```
docker build -t galactic_dev_image .
```
and then run the container using docker compose:
```
docker compose up -d
```
and then
```
docker exec -it galactic_dev_container bash
```

You can now edit the local src files and create your packages inside the src folder on your host machine and it will sync inside the docker container, and in order to run the nodes or build them, access the running docker container terminal and colcon build and run the nodes like you normally do.
