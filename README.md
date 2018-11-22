# `mdr_erl_scenarios`
package for interfacing between the Central Factory Hub (CFH) and b-it-bots@Home repositories, using
[`mas_execution_manager`](https://github.com/b-it-bots/mas_execution_manager) for state machine definition and
execution.

## Starting a CFH client instance
The file [`docker-compose.yml`](docker/docker-compose.yml) under the `docker` directory provide necessary configurations
for starting up a CFH client container.
* Before running the the container, [`robot_example_ros.launch`](docker/robot_example_ros.launch) needs to be updated:
    - `host_name` needs to be the correct IP address for the CFH server
    - `robot_name` and `team_name` needs to be the correct values
* To start the container, run with admin privilege:
    ```bash
    docker-compose up -f docker-compose.yml
    ```