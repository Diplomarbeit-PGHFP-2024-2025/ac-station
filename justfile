set shell := ["bash", "-c"]

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    rm -rd venv/lib/python3.12/site-packages/aca_protocols
    rm -rd venv/lib/python3.12/site-packages/aca_protocols-0.1.0.dist-info
    python3 -m pip install uagents==0.17.0
    python3 -m pip install git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@6271194afccac3738a185a51786f71ebf5ee18c8
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

all:
    just install
    just fix
    just build
    just run-agent
