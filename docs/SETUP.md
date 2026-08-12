# MarsCommonFtc Setup Guide

Shared FTC modules consumed as a git submodule: **ControlLib**, **ControlLab**, **WpiMath**,
plus nested **[RuckigJava](https://github.com/Mars-Robotics-Association/RuckigJava)** (pure-Java Ruckig OTG).

## Prerequisites

- Git (with submodule support)
- Android Studio (Ladybug or later recommended)
- JDK 17

## Pre-commit formatting (contributors)

This repo uses the same git hook approach as `mars-road-runner-quickstart`: staged
`.java` files are reformatted with **google-java-format** (AOSP / 4-space) and staged
`.kt` files with **ktfmt** (`--kotlinlang-style` / 4-space). Formatters re-stage the
files before the commit completes.

Once per clone:

```bash
git config core.hooksPath .githooks
```

Pinned jars live under `~/.githooks` (shared across clones):

| Tool | Version | Path |
|------|---------|------|
| google-java-format | 1.28.0 | `~/.githooks/google-java-format.jar` |
| ktfmt | 0.64 | `~/.githooks/ktfmt.jar` |

If a jar is missing, the hook prints the install `curl` command. Example:

```bash
mkdir -p "$HOME/.githooks"
curl -fsSL -o "$HOME/.githooks/ktfmt.jar" \
  "https://github.com/Kotlin/ktfmt/releases/download/v0.64/ktfmt-0.64-with-dependencies.jar"
curl -fsSL -o "$HOME/.githooks/google-java-format.jar" \
  "https://github.com/google/google-java-format/releases/download/v1.28.0/google-java-format-1.28.0-all-deps.jar"
```

Requires Java 17+ on `PATH` or `JAVA_HOME`. Override jar locations with
`GOOGLE_JAVA_FORMAT_JAR` / `KTFMT_FORMAT_JAR` if needed.

## Adding to Your Robot Project

### SourceTree

1. Open your robot project repository
2. Go to **Repository → Add Submodule**
3. Source URL: `https://github.com/Mars-Robotics-Association/MarsCommonFtc.git`
4. Local path: `MarsCommonFtc`
5. After adding, right-click the submodule → **Open Submodule** → checkout the desired tag (e.g. `v1.0.0`)
6. Inside MarsCommonFtc, init nested submodules (**Repository → Submodule Update** / update recursive) so `MarsCommonFtc/RuckigJava` is populated

### CLI

```bash
git submodule add https://github.com/Mars-Robotics-Association/MarsCommonFtc.git MarsCommonFtc
cd MarsCommonFtc && git checkout v1.0.0
git submodule update --init --recursive
cd ..
git add MarsCommonFtc .gitmodules
git commit -m "add MarsCommonFtc submodule at v1.0.0"
```

Expected layout:

```text
YourRobotProject/
  MarsCommonFtc/              # submodule
    RuckigJava/               # nested submodule (init with --recursive)
    ControlLib/
    ControlLab/
    WpiMath/
  TeamCode/
  ...
```

## settings.gradle Configuration

Add these lines to your project's `settings.gradle` (after your own module includes):

```groovy
// --- MarsCommonFtc modules ---
include ':WpiMath'
project(':WpiMath').projectDir = file('MarsCommonFtc/WpiMath')

include ':ControlLib'
project(':ControlLib').projectDir = file('MarsCommonFtc/ControlLib')

include ':ControlLab'
project(':ControlLab').projectDir = file('MarsCommonFtc/ControlLab')

include ':RuckigJava'
project(':RuckigJava').projectDir = file('MarsCommonFtc/RuckigJava')
```

Your existing dependency declarations (e.g. `implementation project(':ControlLib')`) will work without changes since the project names are the same.

## Updating to a New Version

### SourceTree

1. Open the MarsCommonFtc submodule in SourceTree
2. Fetch all remotes
3. Checkout the new tag (e.g. `v1.1.0`)
4. Update nested submodules if the pin changed
5. Go back to the parent project and commit the submodule pointer change

### CLI

```bash
cd MarsCommonFtc
git fetch
git checkout v1.1.0
git submodule update --init --recursive
cd ..
git add MarsCommonFtc
git commit -m "update MarsCommonFtc to v1.1.0"
```

## Troubleshooting

### "Could not find :ControlLib" (or similar)

Make sure your `settings.gradle` includes the `projectDir` redirects shown above. The project names must match exactly (`:ControlLib`, `:WpiMath`, `:RuckigJava`).

### "Could not find :RuckigJava" / empty `MarsCommonFtc/RuckigJava`

Init nested submodules:

```bash
cd MarsCommonFtc
git submodule update --init --recursive
```

Confirm `MarsCommonFtc/RuckigJava/build.gradle` exists and that `settings.gradle` points at `MarsCommonFtc/RuckigJava`.
