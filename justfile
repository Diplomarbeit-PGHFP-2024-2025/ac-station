set shell := ["bash", "-c"]

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    python3 -m pip install --ignore-requires-python uagents==0.19.0
    python3 -m pip install --ignore-requires-python git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@ea16602378e2d002b756fee205e67632857f18aa
    python3 -m pip install ruff
    python3 -m pip install python-dotenv

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
