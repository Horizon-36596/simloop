# API reference

Every public class, method and field in `org.horizon36596.simloop`, with **units and frame stated on
every number**.

<a href="../javadoc/index.html" class="md-button md-button--primary">Open the API reference</a>

That link opens the generated Javadoc beside this guide, both on the published site and in a full
local build. Build the guide and API reference together from the repository root:

```powershell
./gradlew :SimLoop:docsSite
```

Then open `SimLoop/build/docs/site/api-reference/index.html`. Running MkDocs alone builds the guide
but does not copy the API pages. For Javadoc alone, run `:SimLoop:apiDocs` and open
`SimLoop/build/docs/apiDocs/index.html`.

(Run it from the repository root, not from inside `SimLoop/`. The library is a Gradle subproject of a
repository that holds only it, so the `:SimLoop` prefix is correct.)

## How it is kept complete

Javadoc is generated with `-Xdoclint:all` and `-Werror`. Missing comments, parameters, return
descriptions and broken `{@link}` references fail the build. Units and coordinate frames belong in
the public API comments so callers do not have to infer them from a method name.

The same build ships in every release as `SimLoop-<version>-javadoc.jar`, so the pages your IDE shows and
the pages on this site are the same build.
