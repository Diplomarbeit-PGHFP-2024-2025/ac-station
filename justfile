set shell := ["bash", "-c"]

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    python3 -m pip install uagents==0.17.1
    python3 -m pip install git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@9839bb490e06d4aadb3b66d2a35062360d4dd62d
    python3 -m pip install ruff

lint:
    ruff check
    ruff format --check

fix:
    ruff check --fix
    ruff format

build:
    colcon build

run-agent:
    ros2 run fetch_agent agent

build-run-agent:
    just build
    just run-agent
