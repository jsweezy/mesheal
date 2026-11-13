# Docker

The repository includes a `Dockerfile` that builds Mesheal on Ubuntu 24.04, runs the full test
suite, and produces a small image with the `mesheal` command-line tool. Use it to:

- run Mesheal's [command line](CLI.md) on Windows, Linux or macOS without installing a compiler
  or libraries;
- check that Mesheal builds and passes its tests on a clean Linux system.

!!! note "Command line only"
    The Docker image runs headless actions such as `--diagnostics`, `--fix-all` and the remeshers.
    It does not open the desktop app: for that, [download](install.md#download) Mesheal for
    Windows, macOS or Linux, or [build it from source](install.md#build-from-source).

## How it works

Two Docker terms are useful here:

- An **image** is a packaged, read-only Linux system with Mesheal installed. `docker build`
  creates it once; it stays on your disk until you delete it. Rebuild it only to update Mesheal.
- A **container** is a running copy of an image. `docker run --rm ...` starts one, runs a single
  `mesheal` command, and deletes the container when the command finishes. Nothing stays running.

The only thing that must be running is the Docker engine itself (Docker Desktop on Windows and
macOS).

## Build the image

Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) (Windows, macOS) or
Docker Engine (Linux), then from the repository folder:

```bash
docker build -t mesheal .
```

The first build downloads and compiles every dependency and runs the test suite: expect 15 to 30
minutes. If any test fails, the build stops.

Later builds are much faster. Docker caches each step: the Ubuntu packages are installed once, and
the build folder is kept in a cache between builds, so only files that changed are recompiled.

!!! tip "Compile jobs and memory"
    The image compiles one file at a time by default, because some files need about 2 GB of memory
    each and Docker Desktop's virtual machine often has only 4 GB. If you have given Docker more
    memory (Docker Desktop: **Settings → Resources**), compile in parallel with, for example,
    `docker build --build-arg JOBS=4 -t mesheal .` (roughly 2 GB per job).

## Run it

Mount the folder that holds your STL files at `/data` and pass the same arguments you would give
`mesheal`:

=== "Linux / macOS"

    ```bash
    docker run --rm -v "$PWD:/data" mesheal part.stl --diagnostics
    docker run --rm -v "$PWD:/data" mesheal part.stl --fix-all -o part_clean.stl
    ```

=== "Windows (PowerShell)"

    ```powershell
    docker run --rm -v "${PWD}:/data" mesheal part.stl --diagnostics
    docker run --rm -v "${PWD}:/data" mesheal part.stl --fix-all -o part_clean.stl
    ```

=== "Windows (Command Prompt)"

    ```bat
    docker run --rm -v "%cd%:/data" mesheal part.stl --diagnostics
    docker run --rm -v "%cd%:/data" mesheal part.stl --fix-all -o part_clean.stl
    ```

`-v` shares your current folder with the container as `/data`; a container cannot see your disk
otherwise. Output files are written into that folder, next to your input. `docker run --rm mesheal`
with no arguments prints the help.

To type less, define a shell alias (Linux / macOS):

```bash
alias mesheal-docker='docker run --rm -v "$PWD:/data" mesheal'
mesheal-docker part.stl --fix-all
```

## Build and test only

To compile and run the tests without producing the runtime image (this is what the project's
continuous integration does on every change):

```bash
docker build --target build -t mesheal-build .
```

The build stage compiles the full desktop app too, so it also checks that the GUI builds on
Linux. The runtime image ships only `mesheal-cli`, the command-line-only executable, which needs
no OpenGL or X11 libraries.

Build arguments:

| Argument         | Default | Meaning                    |
|------------------|---------|----------------------------|
| `JOBS`           | `1`     | Parallel compile jobs      |
| `TEST_JOBS`      | `4`     | Parallel test jobs         |
| `UBUNTU_VERSION` | `24.04` | Ubuntu release to build on |

## Continuous integration

`.github/workflows/linux.yml` builds the `build` stage on every push and pull request (with
`JOBS=4` on GitHub's runners), so a change that breaks the Linux build or a test fails the check.
It then builds the runtime image and runs `--help` and `--diagnostics` on a bundled example.

## Faster rebuilds while developing

Rebuilding the image copies the whole source tree each time. When fixing a Linux-only build
problem, it is quicker to compile inside a container that sees your working copy directly and
keeps its build folder in a named volume:

```bash
docker build --target deps -t mesheal-deps .
docker run --rm -v "$PWD:/src:ro" -v mesheal-linux-build:/build mesheal-deps sh -c \
  'cmake -S /src -B /build -G Ninja -DCMAKE_BUILD_TYPE=Release &&
   cmake --build /build -j 1 -- -k 0 &&
   ctest --test-dir /build -j 4 --output-on-failure'
```

`-- -k 0` makes Ninja keep going after an error, so one run lists every file that fails to
compile. Run the same command again after each fix; only changed files are recompiled.

## Cleaning up

Docker keeps images, build cache and volumes until you remove them. They can take several GB.

```bash
docker image ls                      # list images and their sizes
docker image rm mesheal mesheal-build mesheal-deps   # remove Mesheal's images
docker volume rm mesheal-linux-build # remove the development build folder (if you used it)
docker builder prune                 # remove Docker's build cache, including the kept build folder
docker system df                     # show how much space Docker is using
```

After `docker builder prune`, the next `docker build` starts from scratch.
