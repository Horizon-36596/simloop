# Installing it

Copy the starter folder to add SimLoop to an FTC project. The manual setup below explains the same
build settings if you want to maintain them yourself.

!!! warning "Your project must be on FTC SDK 12.0"
    SimLoop `v0.2.0-alpha.1` is built against **FTC SDK 12.0.0**, and the earlier beta was built against
    11.2.1. Update your FTC project to SDK 12.0 before you add SimLoop. See
    [Why SDK 12.0](#why-sdk-120) below.

!!! note "Status: alpha"
    `v0.2.0-alpha.1` is the current alpha. The API may change between alpha versions, so pin the exact
    version. `v0.1.0-beta1` is superseded and gets no fixes. Questions go to
    [Discussions](https://github.com/Horizon-36596/simloop/discussions).

## What you need first

- **A JDK compatible with your project's Gradle wrapper.** Java 17 is the baseline for this build;
  Android Studio includes a JDK, but its version must still match what your wrapper supports.
- **An FTC SDK 12.0 project.** `RobotCore` and `Hardware` at `12.0.0`.
- **The Android SDK required by that project**, installed and configured for Gradle. A plain JVM test
  still builds through the Android Gradle Plugin; it does not need an Android emulator.

You do **not** need to clone this repository, and you do not need SolversLib,
Pedro Pathing, Road Runner, FTC Dashboard or Robolectric. SimLoop depends on none of them.

### Why SDK 12.0

SimLoop's fakes **are** SDK types: `FakeMotor` is a `DcMotorEx`, and the goBILDA Pinpoint, SparkFun OTOS
and Limelight 3A fakes are the SDK 12.0 driver classes themselves. SimLoop passes the SDK through to your
build at compile scope. If your project pins 11.x, Gradle quietly promotes your test classpath to 12.0
while your robot code is compiled against 11.x, so your tests run against a different SDK from the one
your robot runs. Update the project first.

## The three-step way: copy the starter folder

[`simloop-starter/`](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.1/simloop-starter) is one folder
that carries every build setting SimLoop needs, plus a small example robot that drives itself.

1. **Copy the folder** into the root of your FTC project, beside `TeamCode/` and `FtcRobotController/`.
2. **Add one line to the bottom of `TeamCode/build.gradle`**, after the `android { }` block:

    ```groovy
    apply from: "$rootDir/simloop-starter/simloop.gradle"
    ```

3. **Run it.**

    ```powershell
    ./gradlew :TeamCode:testDebugUnitTest
    ```

The two starter tests should pass without a robot or Driver Station. On the first run, Gradle may
need to download dependencies or wait for JitPack to build the pinned tag. Later runs use the cache.

Copy the starter from the same tag as the dependency it declares. The repository's `main` branch can
contain examples for APIs that have not been released. After the starter passes, add a test for your
own robot; passing the example checks the installation, not your robot's configuration or physics.

!!! tip "Why a folder and not a quickstart repository"
    A team starts its season in one repository. A folder you drop into that repository needs no merge,
    and removing it is deleting the folder and one line.

The starter's own [README](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.1/simloop-starter#readme)
says what is in it. [Your first simulated test](first-test.md) walks through a separate slide example,
including the clock, plant, log and assertions.

## The manual way: what the starter folder is actually doing

Read this if you would rather wire it yourself, or if you want to know what you just applied.

### The repository lines

Two of them, and **both are required**. Add them where your project declares repositories — in an FTC
project that is the `allprojects { repositories { } }` block of the root `build.gradle`, or a
`repositories { }` block in `TeamCode/build.gradle`, which is where the starter puts them.

```groovy
repositories {
    mavenCentral()
    google()

    // 1. JitPack, which builds a tag of the SimLoop repository on demand.
    maven { url = 'https://jitpack.io' }

    // 2. Dairy, where PsiKit lives. NOT optional - see below.
    maven { url = 'https://repo.dairy.foundation/releases' }
}
```

!!! note "Why the Dairy line is not optional"
    SimLoop depends on PsiKit, and PsiKit is not on Maven Central. A POM can tell your build **what** a
    dependency is; Gradle resolves it from your project's repository list. Without the Dairy line,
    resolution fails with `Could not find org.psilynx.psikit:core`.

### The dependency

```groovy
// TeamCode/build.gradle
dependencies {
    testImplementation 'com.github.Horizon-36596:simloop:v0.2.0-alpha.1'

    // The keyboard window for driving the sim live. Optional; same coordinate, `desktop` classifier.
    testImplementation 'com.github.Horizon-36596:simloop:v0.2.0-alpha.1:desktop'

    // JUnit 5, if you are not already on it.
    testImplementation 'org.junit.jupiter:junit-jupiter-api:5.11.3'
    testRuntimeOnly 'org.junit.jupiter:junit-jupiter-engine:5.11.3'
    testRuntimeOnly 'org.junit.platform:junit-platform-launcher:1.11.3'
}

android {
    testOptions {
        unitTests.all {
            useJUnitPlatform()
        }
    }
}
```

`testImplementation`, not `implementation`. SimLoop is for the tests that run on your laptop; nothing in
it belongs on the robot, and this scope is what keeps it out of the APK.

!!! warning "The version string includes the `v`"
    JitPack's version **is** the git tag, letter for letter. The tag is `v0.2.0-alpha.1`, so the version is
    `v0.2.0-alpha.1`. Omitting the `v` requests a different version and can fail with `Could not find`.

!!! tip "Declare the JUnit Platform launcher"
    Gradle 9 removed the implicit test-framework dependencies that older Gradle builds could rely on.
    Declare the launcher explicitly to avoid `Failed to load JUnit Platform`. See
    [Gradle's test-runtime dependency guidance](https://docs.gradle.org/current/userguide/java_testing.html#sec:test_execution).

### Avoid two test variants writing the same log

The starter turns off the **release** unit-test variant:

```groovy
androidComponents {
    beforeVariants(selector().withBuildType('release')) { variantBuilder ->
        variantBuilder.enableUnitTest = false
    }
}
```

Android creates unit-test tasks for each build type. `./gradlew test` can run both debug and release
tasks, potentially in separate JVMs. The starter examples use fixed paths in `build/sim/`; two tasks
writing the same path can corrupt or truncate a log, or fail to delete an open file.

The starter needs only the debug tests. If your project has variant-specific code or dependencies
and needs both variants tested, keep both enabled and give each variant distinct log paths. Also
avoid overlapping runs of the same scenario. A test that passes alone but fails alongside another
writer is a reason to check file ownership before changing the model.

### Which scope SimLoop's own dependencies arrive at

SimLoop declares the FTC SDK (`RobotCore`, `Hardware`) and PsiKit core as `api`, so those dependencies
reach **your test compile classpath** too. Public signatures expose SDK types; PsiKit core is also
available so your scenario can call `Logger.recordOutput(...)` without declaring a second version.

Those are SimLoop's three direct dependencies; they can bring transitive dependencies of their own.

### Two test settings some fakes need

The starter folder sets both of these for you. If you install by hand instead, add them yourself:

```groovy
android {
    testOptions {
        unitTests.returnDefaultValues = true   // FakeImu: the SDK's orientation types call an Android method
    }
}

dependencies {
    testRuntimeOnly "org.json:json:20250517"   // FakeLimelight3A: Android's org.json is a stub in JVM tests
}
```

Without the first, `new RevHubOrientationOnRobot(...)` throws "not mocked". Without the second, every
fake Limelight result is empty or throws.

## Building it yourself

Clone the source to test an unreleased change or build APIs absent from the published tag. Run from
the repository root:

```powershell
./gradlew :SimLoop:publishToMavenLocal
```

Then point your project at that local repository, *before* the others so it wins:

```groovy
repositories {
    mavenLocal()
    mavenCentral()
    google()
    maven { url = 'https://repo.dairy.foundation/releases' }
}
```

```groovy
dependencies {
    testImplementation 'org.horizon36596:SimLoop:0.2.0-alpha.1'
}
```

!!! note "Two coordinates for one library, and both are correct"
    `org.horizon36596:SimLoop:0.2.0-alpha.1` is what this project declares, and it is what
    `publishToMavenLocal` writes and what a future Maven Central release would use. **JitPack ignores
    all three parts and derives its own** from the repository address: the group becomes
    `com.github.Horizon-36596`, the artifact becomes the repository name `simloop`, and the version is
    the tag verbatim — so a JitPack dependency is spelled `com.github.Horizon-36596:simloop:v0.2.0-alpha.1`.

    Use the coordinate for the repository you are resolving from.

## Checking it worked

If you copied the starter folder, confirm that both example tests ran and passed. If you wired the
build by hand, add the [first simulated test](first-test.md), then run:

```powershell
./gradlew :TeamCode:testDebugUnitTest
```

A successful task with zero tests does not verify the installation. Check the test report and test
source path; after adding the slide example, the report must show `SlideReachesTargetExampleTest`.
Open `TeamCode/build/reports/tests/testDebugUnitTest/index.html` for the per-test results.
