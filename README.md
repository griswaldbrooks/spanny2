[![ci](https://github.com/griswaldbrooks/spanny2/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny2/actions/workflows/ci.yml)
# spanny2
Robot arm project for CppCon 2024 presentation.

# development container
Build a new development image
```shell
mkdir -p ~/.spanny2/ccache
export UID=$(id -u) export GID=$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
cmake -S src/spanny2/ -B build
cmake --build build
```

# run
```shell
username@spanny-dev:~/ws$ ./build/mappy
```

# remove orphaned containers
```shell
docker compose -f compose.dev.yml down --remove-orphans
```
