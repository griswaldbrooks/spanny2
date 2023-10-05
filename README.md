# spanny
Robot arm project for lightening talk on mdspan.
Presented as a ligthening talk at CppCon 2023.

# Development Container
Build a new development image
```shell
mkdir -p ~/.spanny/ccache
export UID=$(id -u) export GID=$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
username@spanny-dev:~/ws$ cmake -S src/spanny/ -B build
username@spanny-dev:~/ws$ cmake --build build
```

# Run
```shell
username@spanny-dev:~/ws$ ./build/spanny
```
