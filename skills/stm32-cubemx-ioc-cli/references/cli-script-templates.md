# CLI Script Templates

Use absolute paths in every command.

## A. Daily regeneration (`generate code`)

Use this only when generation output and build-consumed source tree are the same.

```txt
config load /ABS/PATH/TO/project.ioc
generate code /ABS/PATH/TO/project_root
exit
```

## B. Full project regeneration (`project generate`)

```txt
config load /ABS/PATH/TO/project.ioc
project toolchain CMake
project name ProjectName
project generate
exit
```

Optional:
- `project path /ABS/PATH/TO/project_root` (some CubeMX versions may return `KO`; omit if so)
- If `project compiler` is unsupported by current CubeMX version, omit it.

## B-1. Layout mismatch fallback (recommended)

If `generate code` writes to `Src/Inc` but build consumes `Core/Src` and `Core/Inc`, use section B script directly.

## C. Execution example

```bash
/ABS/PATH/TO/STM32CubeMX -q /ABS/PATH/TO/cube_headless.txt
```

## D. Build verification example

```bash
rm -rf /ABS/PATH/TO/project_root/build/Debug
cmake --preset Debug
cmake --build /ABS/PATH/TO/project_root/build/Debug
```
