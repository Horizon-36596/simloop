# Documentation plot sources

`SimLoop/docs/assets/first-test-slide.svg` is a plot of an actual example test run, not a drawing of an
expected trajectory. Its numeric source is `first-test-slide.csv` beside it. The data was recorded on
2026-10-24 from `SlideReachesTargetExampleTest` at source commit `4b40ef9`, using its placeholder plant
parameters, 150 ticks at 0.02 s and a 12 in target. It is simulated data, not a robot measurement.

To refresh it, first run the example from the repository root:

```powershell
./gradlew :SimLoop:testDebugUnitTest --tests org.horizon36596.simloop.example.SlideReachesTargetExampleTest
```

`ExportSlide.java` uses SimLoop's own `RlogTimeSeries` reader to export the recorded position. Run it
with a JDK that supports source-file execution. Its classpath needs the compiled SimLoop classes and
the pinned PsiKit core jar. On Windows, for the current Android Gradle Plugin layout:

```powershell
$docsClasses = (Resolve-Path 'SimLoop/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes').Path
# Set this to the resolved org.psilynx.psikit:core:0.2.0-beta2 jar in your Gradle cache.
$docsPsiKitJar = '<absolute path to core-0.2.0-beta2.jar>'
java --class-path "$docsClasses;$docsPsiKitJar" tools/ExportSlide.java `
  SimLoop/build/sim/slide-reaches-target.rlog SimLoop/docs/assets/first-test-slide.csv
python tools/plot-first-test.py
```

The plotting script needs Matplotlib in the Python environment used to run it. It is an optional
authoring tool; the documentation build uses the committed SVG and does not run Python plotting or
change the library's dependencies. On other systems use `:` between Java classpath entries.

After regeneration, verify that there are 150 rows, that the final value is within 0.25 in of 12 in,
and that the peak is at most 14 in, matching the example's assertions. Update the source commit above
if the example changed, and inspect the plot before committing it.
