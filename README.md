# Flecs Traffic
Traffic simulation written in Flecs ([live wasm demo](https://flecs-hub.github.io/traffic/etc/)). Work in progress!

## How to run
Use the following commands to run the demo:

Install bake on macOS/Linux:
```
git clone https://github.com/SanderMertens/bake
bake/setup.sh
```

Install bake on Windows (don't use powershell)
```
git clone https://github.com/SanderMertens/bake
cd bake
setup.bat
```

or if you already have a bake installation:
```
bake upgrade
```

Clone the repository:
```
git clone https://github.com/SanderMertens/traffic
```

and run:
```
cd traffic
bake run --cfg release
```

To increase/decrease the size of the grid, modify this file: https://github.com/flecs-hub/traffic/blob/main/etc/assets/main.flecs

Have fun!
